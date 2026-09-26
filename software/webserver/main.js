import * as THREE from 'three'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import './style.css'
import { isProtobufRecording, parseRecording } from './recording.js'

const $ = id => document.getElementById(id)
const DEG = 180 / Math.PI
let currentStatus = null
let recording = null
let selected = null
let playing = null
let charts = []
let polling = false
let timeWindow = null
const dirtyControls = new Set()

async function api(path, options) {
  const response = await fetch(`/api${path}`, { cache: 'no-store', ...options })
  if (!response.ok) {
    let message = `${response.status} ${response.statusText}`
    try { message = (await response.json()).message || message } catch { /* proxy errors may be plain text */ }
    throw new Error(message)
  }
  return response
}

async function json(path, options) { return (await api(path, options)).json() }

function note(id, message, error = false) {
  const el = $(id)
  el.textContent = message
  el.classList.toggle('error', error)
}

const scene = new THREE.Scene()
const camera = new THREE.PerspectiveCamera(38, 1, .1, 100)
camera.position.set(4, 2.8, 5.5)
camera.lookAt(0, .7, 0)
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
renderer.setPixelRatio(Math.min(devicePixelRatio, 2))
$('robot-view').append(renderer.domElement)
scene.add(new THREE.HemisphereLight(0xc5fff0, 0x193539, 2.2))
const light = new THREE.DirectionalLight(0xffffff, 2.5)
light.position.set(-3, 5, 4)
scene.add(light)
const robot = new THREE.Group()
scene.add(robot)
const body = new THREE.Mesh(new THREE.BoxGeometry(1.05, 1.55, .34), new THREE.MeshStandardMaterial({ color: 0x79dcb6, metalness: .35, roughness: .35 }))
body.position.y = .94
robot.add(body)
const face = new THREE.Mesh(new THREE.BoxGeometry(.72, .46, .02), new THREE.MeshStandardMaterial({ color: 0x24474a }))
face.position.set(0, 1.22, .18)
robot.add(face)
for (const x of [-.68, .68]) {
  const wheel = new THREE.Mesh(new THREE.CylinderGeometry(.43, .43, .2, 32), new THREE.MeshStandardMaterial({ color: 0x26363b, roughness: .75 }))
  wheel.rotation.z = Math.PI / 2
  wheel.position.set(x, .43, 0)
  robot.add(wheel)
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(.16, .16, .21, 24), new THREE.MeshStandardMaterial({ color: 0xa3e6c7, metalness: .5 }))
  hub.rotation.z = Math.PI / 2
  hub.position.set(x, .43, 0)
  robot.add(hub)
}
const ground = new THREE.Mesh(new THREE.PlaneGeometry(20, 20), new THREE.MeshBasicMaterial({ color: 0x2b5354, transparent: true, opacity: .22 }))
ground.rotation.x = -Math.PI / 2
scene.add(ground)
function resizeScene() {
  const host = $('robot-view')
  const width = host.clientWidth
  const height = host.clientHeight
  camera.aspect = width / height
  camera.updateProjectionMatrix()
  renderer.setSize(width, height)
  renderer.render(scene, camera)
}
new ResizeObserver(resizeScene).observe($('robot-view'))
function showPitch(rad, source) {
  robot.rotation.x = Number.isFinite(rad) ? rad : 0
  $('view-source').textContent = source
  renderer.render(scene, camera)
}

function updateControls(status) {
  const form = $('control-form')
  const values = {
    PID_Kp: status.PID_Kp, PID_Ki: status.PID_Ki, PID_Kd: status.PID_Kd,
    PID_setpoint: Number(status.PID_setpoint) * DEG,
    MOTOR_DUTY_CYCLE_MAX: status.MOTOR_DUTY_CYCLE_MAX,
    CONTROL_MODE: status.CONTROL_MODE,
  }
  form.elements.namedItem('MOTOR_DUTY_CYCLE_MAX').min = status.MOTOR_DUTY_CYCLE_MIN
  for (const [key, value] of Object.entries(values)) {
    const input = form.elements.namedItem(key)
    if (!dirtyControls.has(key) && document.activeElement !== input && value != null) input.value = typeof value === 'number' ? Number(value.toFixed(5)) : value
  }
}

function updateStatus(status) {
  currentStatus = status
  $('connection').textContent = 'Robot connected'
  $('connection-dot').classList.add('online')
  $('live-pitch').textContent = Number.isFinite(Number(status.pitch_angle_current)) ? `${(Number(status.pitch_angle_current) * DEG).toFixed(1)}°` : '—°'
  $('mode-chip').textContent = `Mode ${status.CONTROL_MODE || '—'}`
  $('estimate-chip').textContent = `Estimate ${status.ESTIMATES_VALID ? 'valid' : 'invalid'}`
  $('arm-chip').textContent = `Auto arm ${status.AUTO_ARM_ALLOWED ? 'allowed' : 'blocked'}`
  if (selected === null) showPitch(Number(status.pitch_angle_current), 'Live robot pitch')
  updateControls(status)
  if (!$('control-message').dataset.pending) note('control-message', 'Controller values are current.')
}

function updateRecording(status) {
  $('recording-state').textContent = status.state || '—'
  const count = Number(status.record_count) || 0
  const capacity = Number(status.capacity) || 0
  $('progress-fill').style.width = `${capacity ? Math.min(100, count / capacity * 100) : 0}%`
  $('recording-progress').textContent = `${count.toLocaleString()} / ${capacity.toLocaleString()} samples · ${Number(status.duration_seconds || 0).toFixed(1)} / ${Number(status.max_duration_seconds || 0).toFixed(1)} s`
  $('start-recording').disabled = status.state === 'DOWNLOADING' || status.state === 'RECORDING'
  $('stop-recording').disabled = status.state !== 'RECORDING'
  $('download-recording').disabled = !status.download_ready
}

async function poll() {
  if (polling) return
  polling = true
  try {
    const [status, rec] = await Promise.all([json('/status'), json('/recording/status')])
    updateStatus(status)
    updateRecording(rec)
  } catch (error) {
    $('connection').textContent = `Robot unavailable: ${error.message}`
    $('connection-dot').classList.remove('online')
    $('recording-state').textContent = 'OFFLINE'
    for (const id of ['start-recording', 'stop-recording', 'download-recording']) $(id).disabled = true
    note('control-message', 'Robot connection unavailable.', true)
  } finally { polling = false }
}
poll()
setInterval(poll, 1500)

async function sendValue(key, value) {
  const body = new URLSearchParams({ key, value: String(value) })
  return json('/set-value', { method: 'POST', body })
}

for (const input of $('control-form').querySelectorAll('input, select')) {
  input.addEventListener('input', () => dirtyControls.add(input.name))
  input.addEventListener('change', () => dirtyControls.add(input.name))
}

async function confirmValues(expected) {
  for (let attempt = 0; attempt < 8; attempt++) {
    await new Promise(resolve => setTimeout(resolve, 250))
    const status = await json('/status')
    updateStatus(status)
    if (expected.every(([key, value]) => key === 'CONTROL_MODE' ? status[key] === value : Math.abs(Number(status[key]) - Number(value)) < 1e-4)) return true
  }
  return false
}

$('control-form').addEventListener('submit', async event => {
  event.preventDefault()
  if (!currentStatus) return note('control-message', 'Robot is not connected.', true)
  const button = $('control-form').querySelector('button')
  const form = new FormData(event.currentTarget)
  const expected = []
  for (const key of ['PID_Kp', 'PID_Ki', 'PID_Kd', 'PID_setpoint', 'MOTOR_DUTY_CYCLE_MAX', 'CONTROL_MODE']) {
    let value = form.get(key)
    if (key === 'PID_setpoint') value = Number(value) / DEG
    if (key !== 'CONTROL_MODE' && !Number.isFinite(Number(value))) return note('control-message', `Invalid ${key}.`, true)
    const unchanged = key === 'CONTROL_MODE' ? value === currentStatus[key] : Math.abs(Number(value) - Number(currentStatus[key])) < 1e-5
    if (!unchanged) expected.push([key, value])
  }
  if (!expected.length) {
    dirtyControls.clear()
    updateControls(currentStatus)
    return note('control-message', 'No values changed.')
  }
  button.disabled = true
  $('control-message').dataset.pending = '1'
  try {
    for (const [key, value] of expected) {
      note('control-message', `Queueing ${key}…`)
      await sendValue(key, value)
    }
    note('control-message', 'Waiting for robot status confirmation…')
    const confirmed = await confirmValues(expected)
    if (confirmed) {
      dirtyControls.clear()
      updateControls(currentStatus)
    }
    note('control-message', confirmed ? 'Applied and confirmed from status.' : 'Changes queued; status has not confirmed all values yet.', !confirmed)
  } catch (error) { note('control-message', error.message, true) }
  finally { delete $('control-message').dataset.pending; button.disabled = false }
})

$('emergency').addEventListener('click', async () => {
  try { await sendValue('EMERGENCY_STOP', '1'); note('control-message', 'Emergency stop requested.'); await poll() }
  catch (error) { note('control-message', error.message, true) }
})

for (const [id, path, message] of [
  ['start-recording', '/recording/start', 'Recording start queued.'],
  ['stop-recording', '/recording/stop', 'Recording stop queued.'],
]) {
  $(id).addEventListener('click', async () => {
    $(id).disabled = true
    try { await json(path, { method: 'POST' }); note('recording-message', message); await poll() }
    catch (error) { note('recording-message', error.message, true) }
  })
}

$('download-recording').addEventListener('click', async () => {
  $('download-recording').disabled = true
  try {
    const buffer = await (await api('/recording/download?format=protobuf')).arrayBuffer()
    const protobuf = isProtobufRecording(buffer)
    const blob = new Blob([buffer], { type: protobuf ? 'application/x-protobuf' : 'application/octet-stream' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = protobuf ? 'balance-recording.sbrpb' : 'balance-recording.bin'
    link.click()
    setTimeout(() => URL.revokeObjectURL(url), 60000)
    openRecording(buffer, link.download)
    note('recording-message', 'Recording downloaded and opened.')
  } catch (error) { note('recording-message', error.message, true) }
  finally { await poll() }
})

$('recording-file').addEventListener('change', async event => {
  const file = event.target.files[0]
  if (!file) return
  try { openRecording(await file.arrayBuffer(), file.name) }
  catch (error) { $('file-summary').textContent = error.message; $('file-summary').classList.add('error') }
})

function selectSample(index) {
  if (!recording?.samples.length) return
  selected = Math.max(0, Math.min(index, recording.samples.length - 1))
  const sample = recording.samples[selected]
  $('sample-slider').value = selected
  $('sample-label').textContent = `${sample.elapsedS.toFixed(3)} s · ${sample.pitchDeg.toFixed(2)}° · ${sample.mode}`
  showPitch(sample.pitchRad, `Recording sample ${selected + 1}`)
  for (const chart of charts) chart.setCursor({ left: chart.valToPos(sample.elapsedS, 'x') })
}
$('sample-slider').addEventListener('input', event => selectSample(Number(event.target.value)))
$('live-view').addEventListener('click', () => {
  if (playing) { clearInterval(playing); playing = null; $('playback').textContent = 'Play' }
  selected = null
  $('sample-label').textContent = 'Live'
  showPitch(Number(currentStatus?.pitch_angle_current), 'Live robot pitch')
})
$('playback').addEventListener('click', () => {
  if (playing) { clearInterval(playing); playing = null; $('playback').textContent = 'Play'; return }
  if (selected === recording.samples.length - 1) selectSample(0)
  $('playback').textContent = 'Pause'
  playing = setInterval(() => {
    if (selected >= recording.samples.length - 1) {
      clearInterval(playing); playing = null; $('playback').textContent = 'Play'
    } else selectSample(selected + Math.max(1, Math.round(recording.header.sampleRateHz / 30)))
  }, 33)
})

function recordingDuration() {
  return recording?.samples.length ? recording.samples.at(-1).elapsedS : 0
}

function nearestSampleIndex(time) {
  const samples = recording.samples
  let low = 0
  let high = samples.length - 1
  while (low < high) {
    const middle = Math.floor((low + high) / 2)
    if (samples[middle].elapsedS < time) low = middle + 1
    else high = middle
  }
  if (low > 0 && Math.abs(samples[low - 1].elapsedS - time) < Math.abs(samples[low].elapsedS - time)) return low - 1
  return low
}

function clampTimeWindow(min, max) {
  const duration = recordingDuration()
  if (!Number.isFinite(min) || !Number.isFinite(max) || max <= min || duration <= 0) return [0, duration]
  const span = Math.min(duration, Math.max(1 / recording.header.sampleRateHz, max - min))
  const start = Math.max(0, Math.min(duration - span, min))
  return [start, start + span]
}

function setTimeWindow(min, max) {
  if (!recording || recording.samples.length < 2) return
  const next = clampTimeWindow(min, max)
  timeWindow = next
  const duration = recordingDuration()
  $('time-window-label').textContent = next[0] < 1e-6 && duration - next[1] < 1e-6
    ? `Full recording · 0–${duration.toFixed(2)} s`
    : `Visible · ${next[0].toFixed(2)}–${next[1].toFixed(2)} s`
  $('reset-zoom').disabled = next[0] < 1e-6 && duration - next[1] < 1e-6
  $('pan-left').disabled = next[0] < 1e-6
  $('pan-right').disabled = duration - next[1] < 1e-6
  $('zoom-in').disabled = next[1] - next[0] <= 1 / recording.header.sampleRateHz + 1e-6
  $('zoom-out').disabled = next[1] - next[0] >= duration - 1e-6
  for (const plot of charts) {
    const x = plot.scales.x
    if (Math.abs(x.min - next[0]) > 1e-7 || Math.abs(x.max - next[1]) > 1e-7) plot.setScale('x', { min: next[0], max: next[1] })
  }
  if (selected !== null) {
    for (const plot of charts) plot.setCursor({ left: plot.valToPos(recording.samples[selected].elapsedS, 'x') })
  }
}

function zoomTime(factor, anchor) {
  if (!timeWindow) return
  const [min, max] = timeWindow
  const span = max - min
  const nextSpan = span * factor
  const fraction = Math.max(0, Math.min(1, (anchor - min) / span))
  const nextMin = anchor - fraction * nextSpan
  setTimeWindow(nextMin, nextMin + nextSpan)
}

function panTime(amount) {
  if (timeWindow) setTimeWindow(timeWindow[0] + amount, timeWindow[1] + amount)
}

for (const [id, action] of [
  ['zoom-in', () => zoomTime(.5, (timeWindow[0] + timeWindow[1]) / 2)],
  ['zoom-out', () => zoomTime(2, (timeWindow[0] + timeWindow[1]) / 2)],
  ['pan-left', () => panTime(-(timeWindow[1] - timeWindow[0]) * .25)],
  ['pan-right', () => panTime((timeWindow[1] - timeWindow[0]) * .25)],
  ['reset-zoom', () => setTimeWindow(0, recordingDuration())],
]) $(id).addEventListener('click', action)

function chart(title, fields, samples, encoderToggle = false) {
  const card = document.createElement('div')
  card.className = 'chart-card'
  const heading = document.createElement('h3')
  heading.textContent = title
  const host = document.createElement('div')
  host.className = 'chart'
  if (encoderToggle) {
    const head = document.createElement('div')
    head.className = 'chart-card-head'
    const toggle = document.createElement('button')
    toggle.type = 'button'
    toggle.textContent = 'Zero from start'
    toggle.setAttribute('aria-pressed', 'false')
    head.append(heading, toggle)
    card.append(head, host)
    let zeroed = false
    toggle.addEventListener('click', () => {
      zeroed = !zeroed
      toggle.setAttribute('aria-pressed', String(zeroed))
      const data = [samples.map(sample => sample.elapsedS),
        ...fields.map(([, key]) => samples.map(sample => sample[key] - (zeroed ? samples[0][key] : 0)))]
      plot.setData(data, false)
      plot.setScale('x', { min: timeWindow[0], max: timeWindow[1] })
      if (selected !== null) plot.setCursor({ left: plot.valToPos(recording.samples[selected].elapsedS, 'x') })
    })
  } else card.append(heading, host)
  $('charts').append(card)
  const colors = ['#86e8ba', '#f0ba79', '#8bc7ed']
  const plot = new uPlot({
    width: Math.max(250, host.clientWidth), height: 220,
    scales: { x: { time: false } },
    axes: [{ stroke: '#8baaa8', grid: { stroke: '#274047' }, label: 'Time (s)' }, { stroke: '#8baaa8', grid: { stroke: '#274047' } }],
    series: [{}, ...fields.map(([name], i) => ({ label: name, stroke: colors[i], width: 1.5 }))],
    legend: { show: true },
    cursor: {
      sync: { key: 'sbr-recording', scales: ['x', null], setSeries: false },
      drag: {
        x: true, y: false, dist: 8,
        click: (_plot, event) => event.stopPropagation(),
      },
    },
    hooks: {
      setScale: [(plot, key) => {
        if (key === 'x' && timeWindow && charts.includes(plot)) {
          const { min, max } = plot.scales.x
          if (Math.abs(min - timeWindow[0]) > 1e-7 || Math.abs(max - timeWindow[1]) > 1e-7) setTimeWindow(min, max)
        }
      }],
    },
  }, [samples.map(sample => sample.elapsedS), ...fields.map(([, key]) => samples.map(s => s[key]))], host)
  plot.over.addEventListener('click', event => {
    const x = plot.posToVal(event.clientX - plot.over.getBoundingClientRect().left, 'x')
    selectSample(nearestSampleIndex(x))
  })
  plot.over.addEventListener('wheel', event => {
    event.preventDefault()
    const span = timeWindow[1] - timeWindow[0]
    if (event.shiftKey || Math.abs(event.deltaX) > Math.abs(event.deltaY)) {
      const pixels = event.deltaX || event.deltaY
      panTime(pixels / plot.over.clientWidth * span)
    } else {
      const left = event.clientX - plot.over.getBoundingClientRect().left
      const anchor = plot.posToVal(Math.max(0, Math.min(plot.over.clientWidth, left)), 'x')
      zoomTime(Math.exp(Math.max(-150, Math.min(150, event.deltaY)) * .005), anchor)
    }
  }, { passive: false })
  let dragStart = null
  plot.over.addEventListener('pointerdown', event => {
    if (event.button !== 1) return
    event.preventDefault()
    dragStart = { x: event.clientX, min: timeWindow[0], max: timeWindow[1] }
    plot.over.setPointerCapture(event.pointerId)
  })
  plot.over.addEventListener('pointermove', event => {
    if (!dragStart) return
    const secondsPerPixel = (dragStart.max - dragStart.min) / plot.over.clientWidth
    const offset = (dragStart.x - event.clientX) * secondsPerPixel
    setTimeWindow(dragStart.min + offset, dragStart.max + offset)
  })
  plot.over.addEventListener('pointerup', () => { dragStart = null })
  plot.over.addEventListener('pointercancel', () => { dragStart = null })
  plot.over.addEventListener('auxclick', event => { if (event.button === 1) event.preventDefault() })
  charts.push(plot)
}

function openRecording(buffer, name) {
  const parsed = parseRecording(buffer)
  if (playing) { clearInterval(playing); playing = null; $('playback').textContent = 'Play' }
  for (const plot of charts) plot.destroy()
  charts = []
  timeWindow = null
  $('charts').replaceChildren()
  recording = parsed
  selected = null
  $('file-name').textContent = name
  $('file-summary').classList.remove('error')
  $('file-summary').textContent = `${parsed.header.count.toLocaleString()} samples · ${parsed.header.sampleRateHz} Hz · Kp ${parsed.header.kp.toFixed(3)}, Ki ${parsed.header.ki.toFixed(3)}, Kd ${parsed.header.kd.toFixed(3)} · Setpoint ${(parsed.header.setpointRad * DEG).toFixed(2)}°`
  showRecordingSettings(parsed.header)
  $('sample-slider').max = Math.max(0, parsed.samples.length - 1)
  $('sample-slider').disabled = parsed.samples.length === 0
  $('playback').disabled = parsed.samples.length < 2
  $('sample-label').textContent = '—'
  for (const id of ['zoom-in', 'zoom-out', 'pan-left', 'pan-right', 'reset-zoom']) $(id).disabled = parsed.samples.length < 2
  $('time-window-label').textContent = parsed.samples.length ? `Full recording · 0–${recordingDuration().toFixed(2)} s` : 'No samples'
  if (!parsed.samples.length) return
  chart('Pitch · degrees', [['Pitch', 'pitchDeg']], parsed.samples)
  chart('PID output', [['PID', 'pidOutput']], parsed.samples)
  chart('Gyro · degrees per second', [['Gyro', 'gyroDegS']], parsed.samples)
  chart('Motor PWM · + forward / − backward', [['Motor 1', 'motor1SignedPwm'], ['Motor 2', 'motor2SignedPwm']], parsed.samples)
  chart('Control interval · microseconds', [['Interval', 'controlIntervalUs']], parsed.samples)
  chart('Encoder pulses', [['Motor 1', 'motor1EncoderPulses'], ['Motor 2', 'motor2EncoderPulses']], parsed.samples, true)
  setTimeWindow(0, recordingDuration())
  selectSample(0)
}

function showRecordingSettings(header) {
  const target = $('recording-settings')
  target.replaceChildren()
  const settings = header.settingsAtStart
  if (!settings) {
    target.textContent = 'Metadata unavailable'
    return
  }
  const rows = [
    ['Firmware revision', header.firmwareRevision || 'unknown'],
    ['Settings changed during recording', header.settingsChangedDuringRecording ? 'Yes' : 'No'],
    ['PID gains (Kp / Ki / Kd)', `${settings.pidKp} / ${settings.pidKi} / ${settings.pidKd}`],
    ['PID output limits', `${settings.pidOutputMin} to ${settings.pidOutputMax}`],
    ['PID integral threshold', settings.pidIntegralThreshold],
    ['Pitch setpoint', `${settings.pitchSetpointRad} rad`],
    ['Pitch error min / max', `${settings.pitchErrorMinRad} / ${settings.pitchErrorMaxRad} rad`],
    ['AUTO arm pitch error max', `${settings.autoArmMaxPitchErrorRad} rad`],
    ['Filter gyro weight', settings.complementaryFilterGyroWeight],
    ['Duty min / max', `${settings.dutyCycleMin} / ${settings.dutyCycleMax} PWM counts`],
    ['Motor 1 speed slope / intercept', `${settings.motor1SpeedSlope} / ${settings.motor1SpeedIntercept}`],
    ['Motor 2 speed slope / intercept', `${settings.motor2SpeedSlope} / ${settings.motor2SpeedIntercept}`],
    ['Motor 1 / 2 direction inverted', `${settings.motor1DirectionInverted ? 'Yes' : 'No'} / ${settings.motor2DirectionInverted ? 'Yes' : 'No'}`],
    ['Motor coast', settings.motorCoast ? 'Yes' : 'No'],
    ['PWM frequency / resolution', `${settings.pwmFrequencyHz} Hz / ${settings.pwmResolutionBits} bits`],
    ['Wheel diameter', `${settings.wheelDiameterM} m`],
    ['Encoder pulses / revolution', settings.encoderPulsesPerRevolution],
    ['IMU calibration valid at Start', settings.imuCalibrationValid ? 'Yes' : 'No'],
    ['IMU gyro Y offset', settings.imuCalibrationValid ? `${settings.imuGyroYOffsetRadS} rad/s` : 'Unavailable'],
    ['IMU pitch accel offset', settings.imuCalibrationValid ? `${settings.imuPitchAccelOffsetRad} rad` : 'Unavailable'],
  ]
  const list = document.createElement('dl')
  for (const [label, value] of rows) {
    const term = document.createElement('dt')
    term.textContent = label
    const description = document.createElement('dd')
    description.textContent = String(value)
    list.append(term, description)
  }
  target.append(list)
}
