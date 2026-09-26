import protobuf from 'protobufjs'
import recordingSchema from '../../proto/recording.proto?raw'

const DEG = 180 / Math.PI
const modes = ['AUTO', 'MANUAL', 'FUNCTION', 'UNKNOWN_3']
const schema = protobuf.parse(recordingSchema).root
const RecordingHeader = schema.lookupType('sbr.recording.RecordingHeader')
const RecordingBatch = schema.lookupType('sbr.recording.RecordingBatch')
const PROTO_MAGIC = [83, 66, 82, 80, 66, 49, 0, 0]

export function isProtobufRecording(buffer) {
  const bytes = new Uint8Array(buffer)
  return bytes.length >= PROTO_MAGIC.length && PROTO_MAGIC.every((byte, index) => bytes[index] === byte)
}

function frame(bytes, offset, maxSize) {
  let length = 0
  let shift = 0
  for (let i = 0; i < 5; i++) {
    if (offset >= bytes.length) throw new Error('Truncated protobuf frame length')
    const byte = bytes[offset++]
    length |= (byte & 0x7f) << shift
    if (!(byte & 0x80)) {
      if (length < 1 || length > maxSize) throw new Error(`Invalid protobuf frame length: ${length}`)
      if (offset + length > bytes.length) throw new Error('Truncated protobuf frame')
      return [bytes.subarray(offset, offset + length), offset + length]
    }
    shift += 7
  }
  throw new Error('Protobuf frame length is too long')
}

function parseProtobufRecording(buffer) {
  const bytes = new Uint8Array(buffer)
  let offset = PROTO_MAGIC.length
  let payload
  ;[payload, offset] = frame(bytes, offset, 1024)
  const meta = RecordingHeader.decode(payload)
  if (meta.formatVersion !== 1) throw new Error(`Unsupported recording version: ${meta.formatVersion}`)
  if (meta.sampleRateHz < 1 || meta.sampleRateHz > 10000 || meta.recordCount > meta.capacity) {
    throw new Error('Invalid recording metadata')
  }
  const header = {
    sampleRateHz: meta.sampleRateHz, count: meta.recordCount, capacity: meta.capacity,
    kp: meta.pidKp, ki: meta.pidKi, kd: meta.pidKd,
    setpointRad: meta.pitchSetpointRad, pitchErrorMinRad: meta.pitchErrorMinRad,
    pitchErrorMaxRad: meta.pitchErrorMaxRad, dutyMin: meta.dutyCycleMin,
    dutyMax: meta.dutyCycleMax,
    settingsAtStart: meta.settingsAtStart || null,
    firmwareRevision: meta.firmwareRevision || null,
    settingsChangedDuringRecording: Boolean(meta.settingsChangedDuringRecording),
  }
  const samples = []
  while (samples.length < header.count) {
    ;[payload, offset] = frame(bytes, offset, 2048)
    const batch = RecordingBatch.decode(payload)
    if (batch.firstIndex !== samples.length || batch.samples.length < 1 || batch.samples.length > 16 ||
        samples.length + batch.samples.length > header.count) {
      throw new Error('Invalid recording batch sequence')
    }
    for (const item of batch.samples) {
      const motor1ForwardCommand = Number(item.motor1ForwardCommand)
      const motor2ForwardCommand = Number(item.motor2ForwardCommand)
      samples.push({
        elapsedS: item.elapsedUs / 1e6,
        pitchRad: item.pitchRad, pitchDeg: item.pitchRad * DEG,
        gyroRadS: item.gyroRadS, gyroDegS: item.gyroRadS * DEG,
        pidOutput: item.pidOutput,
        motor1EncoderPulses: item.motor1EncoderPulses,
        motor2EncoderPulses: item.motor2EncoderPulses,
        controlIntervalUs: item.controlIntervalUs,
        motor1Pwm: item.motor1Pwm, motor2Pwm: item.motor2Pwm,
        motor1SignedPwm: motor1ForwardCommand ? item.motor1Pwm : -item.motor1Pwm,
        motor2SignedPwm: motor2ForwardCommand ? item.motor2Pwm : -item.motor2Pwm,
        motor1DirPin: Number(item.motor1DirPin), motor2DirPin: Number(item.motor2DirPin),
        mode: modes[item.mode] || `UNKNOWN_${item.mode}`,
        estimatesValid: Number(item.estimatesValid),
        motor1ForwardCommand, motor2ForwardCommand,
      })
    }
  }
  if (offset !== bytes.length) throw new Error('Recording has trailing data')
  return { header, samples }
}

export function parseRecording(buffer) {
  if (isProtobufRecording(buffer)) {
    return parseProtobufRecording(buffer)
  }
  const view = new DataView(buffer)
  if (view.byteLength < 64) throw new Error('File is too short to contain an SBRLOG1 header')
  const magic = String.fromCharCode(...new Uint8Array(buffer, 0, 8)).replace(/\0+$/, '')
  if (magic !== 'SBRLOG1') throw new Error(`Unexpected file magic: ${magic}`)
  const version = view.getUint16(8, true)
  if (version !== 1) throw new Error(`Unsupported recording version: ${version}`)
  const headerSize = view.getUint16(10, true)
  const recordSize = view.getUint16(12, true)
  if (headerSize !== 64 || recordSize !== 32) throw new Error(`Unsupported layout: header=${headerSize}, record=${recordSize}`)
  const count = view.getUint32(16, true)
  const expectedSize = headerSize + count * recordSize
  if (view.byteLength < expectedSize) throw new Error(`Truncated recording: expected ${expectedSize} bytes, got ${view.byteLength}`)
  const header = {
    sampleRateHz: view.getUint16(14, true), count, capacity: view.getUint32(20, true),
    kp: view.getFloat32(24, true), ki: view.getFloat32(28, true), kd: view.getFloat32(32, true),
    setpointRad: view.getFloat32(36, true), pitchErrorMinRad: view.getFloat32(40, true),
    pitchErrorMaxRad: view.getFloat32(44, true), dutyMin: view.getUint8(48), dutyMax: view.getUint8(49),
    settingsAtStart: null, firmwareRevision: null, settingsChangedDuringRecording: false,
  }
  const samples = Array.from({ length: count }, (_, index) => {
    const o = headerSize + index * recordSize
    const flags = view.getUint8(o + 30)
    const pitchRad = view.getFloat32(o + 4, true)
    const gyroRadS = view.getFloat32(o + 8, true)
    const motor1Pwm = view.getUint8(o + 28)
    const motor2Pwm = view.getUint8(o + 29)
    const motor1ForwardCommand = (flags >> 5) & 1
    const motor2ForwardCommand = (flags >> 6) & 1
    return {
      elapsedS: view.getUint32(o, true) / 1e6,
      pitchRad, pitchDeg: pitchRad * DEG, gyroRadS, gyroDegS: gyroRadS * DEG,
      pidOutput: view.getFloat32(o + 12, true),
      motor1EncoderPulses: view.getInt32(o + 16, true), motor2EncoderPulses: view.getInt32(o + 20, true),
      controlIntervalUs: view.getUint32(o + 24, true),
      motor1Pwm, motor2Pwm,
      motor1SignedPwm: motor1ForwardCommand ? motor1Pwm : -motor1Pwm,
      motor2SignedPwm: motor2ForwardCommand ? motor2Pwm : -motor2Pwm,
      motor1DirPin: flags & 1, motor2DirPin: (flags >> 1) & 1,
      mode: modes[(flags >> 2) & 3], estimatesValid: (flags >> 4) & 1,
      motor1ForwardCommand, motor2ForwardCommand,
    }
  })
  return { header, samples }
}
