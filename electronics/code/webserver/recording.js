const DEG = 180 / Math.PI
const modes = ['AUTO', 'MANUAL', 'FUNCTION', 'UNKNOWN_3']

export function parseRecording(buffer) {
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
