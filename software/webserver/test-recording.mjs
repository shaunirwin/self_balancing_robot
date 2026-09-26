import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import http from 'node:http'
import { spawnSync } from 'node:child_process'
import protobuf from 'protobufjs'
import { createServer } from 'vite'

const schema = protobuf.parse(readFileSync(new URL('../../proto/recording.proto', import.meta.url), 'utf8')).root
const Header = schema.lookupType('sbr.recording.RecordingHeader')
const Batch = schema.lookupType('sbr.recording.RecordingBatch')
const frame = bytes => {
  const prefix = []
  let size = bytes.length
  do {
    prefix.push((size & 127) | (size > 127 ? 128 : 0))
    size >>>= 7
  } while (size)
  return Buffer.concat([Buffer.from(prefix), Buffer.from(bytes)])
}
const magic = Buffer.from([83, 66, 82, 80, 66, 49, 0, 0])
const settings = {
  pidKp: 18, pidOutputMin: -1, pwmFrequencyHz: 30000,
  wheelDiameterM: 0.0618, imuCalibrationValid: false,
}
const makeRecording = (count, extras = {}) => {
  const header = { formatVersion: 1, sampleRateHz: 100, recordCount: count,
    capacity: 2000, pidKp: 18, ...extras }
  const parts = [magic, frame(Header.encode(header).finish())]
  for (let first = 0; first < count; first += 16) {
    const samples = Array.from({ length: Math.min(16, count - first) }, (_, i) => ({
      elapsedUs: (first + i) * 10000, controlIntervalUs: 10000,
      motor1EncoderPulses: -(first + i),
    }))
    parts.push(frame(Batch.encode({ firstIndex: first, samples }).finish()))
  }
  return Buffer.concat(parts)
}

const mock = http.createServer((request, response) => {
  assert.equal(request.url, '/recording/download?format=protobuf')
  const body = makeRecording(2000, { settingsAtStart: settings,
    firmwareRevision: 'abc123', settingsChangedDuringRecording: true })
  response.writeHead(200, { 'Content-Type': 'application/x-protobuf',
    'Content-Length': body.length })
  response.end(body)
})
await new Promise(resolve => mock.listen(0, '127.0.0.1', resolve))
process.env.ROBOT_URL = `http://127.0.0.1:${mock.address().port}`
const vite = await createServer({ server: { host: '127.0.0.1', port: 0,
  strictPort: false } })
try {
  await vite.listen()
  const { parseRecording } = await vite.ssrLoadModule('/recording.js')
  const old = parseRecording(makeRecording(1))
  assert.equal(old.header.settingsAtStart, null)
  const legacy = Buffer.alloc(96)
  legacy.write('SBRLOG1', 0, 'ascii')
  legacy.writeUInt16LE(1, 8)
  legacy.writeUInt16LE(64, 10)
  legacy.writeUInt16LE(32, 12)
  legacy.writeUInt16LE(100, 14)
  legacy.writeUInt32LE(1, 16)
  legacy.writeUInt32LE(2000, 20)
  const oldBinary = parseRecording(legacy.buffer.slice(legacy.byteOffset,
    legacy.byteOffset + legacy.byteLength))
  assert.equal(oldBinary.header.settingsAtStart, null)
  assert.equal(oldBinary.samples.length, 1)
  const unchanged = parseRecording(makeRecording(1, { settingsAtStart: settings }))
  assert.equal(unchanged.header.settingsChangedDuringRecording, false)
  assert.equal(unchanged.header.settingsAtStart.imuCalibrationValid, false)
  const response = await fetch(`http://127.0.0.1:${vite.httpServer.address().port}/api/recording/download?format=protobuf`)
  assert.equal(response.status, 200)
  const bytes = await response.arrayBuffer()
  assert.equal(bytes.byteLength, Number(response.headers.get('content-length')))
  const parsed = parseRecording(bytes)
  assert.equal(parsed.samples.length, 2000)
  assert.equal(parsed.samples.at(-1).elapsedS, 19.99)
  assert.equal(parsed.header.settingsAtStart.pwmFrequencyHz, 30000)
  assert.equal(parsed.header.settingsChangedDuringRecording, true)
  assert.equal(parsed.header.firmwareRevision, 'abc123')
  if (process.env.SBR_PYTHON) {
    const code = `import json, sys
sys.path.insert(0, ${JSON.stringify(new URL('../python/', import.meta.url).pathname)})
from recording_reader import parse_recording
header, samples = parse_recording(sys.stdin.buffer.read())
print(json.dumps({'count': len(samples), 'pwm_hz': header.settings_at_start.pwm_frequency_hz, 'changed': header.settings_changed_during_recording, 'revision': header.firmware_revision}))`
    const python = spawnSync(process.env.SBR_PYTHON, ['-c', code],
      { input: Buffer.from(bytes), encoding: 'utf8' })
    assert.equal(python.status, 0, python.stderr)
    const pythonHeader = JSON.parse(python.stdout)
    assert.deepEqual(pythonHeader, { count: parsed.samples.length,
      pwm_hz: parsed.header.settingsAtStart.pwmFrequencyHz,
      changed: parsed.header.settingsChangedDuringRecording,
      revision: parsed.header.firmwareRevision })
  }
  console.log('Old and new headers, 2,000 samples, and Content-Length through Vite: OK')
} finally {
  await vite.close()
  await new Promise(resolve => mock.close(resolve))
}
