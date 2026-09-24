import * as THREE from 'three'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import './style.css'
import { parseRecording } from './recording.js'

const $ = id => document.getElementById(id)
const DEG = 180 / Math.PI
let currentStatus = null
let recording = null
let selected = null
let playing = null
let charts = []
let polling = false
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
    CONTROL_MODE: status.CONTROL_MODE,
  }
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
  for (const key of ['PID_Kp', 'PID_Ki', 'PID_Kd', 'PID_setpoint', 'CONTROL_MODE']) {
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
    const buffer = await (await api('/recording/download')).arrayBuffer()
    const blob = new Blob([buffer], { type: 'application/octet-stream' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = 'balance-recording.bin'
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
  $('sample-label').textContent = `${(selected / recording.header.sampleRateHz).toFixed(3)} s · ${sample.pitchDeg.toFixed(2)}° · ${sample.mode}`
  showPitch(sample.pitchRad, `Recording sample ${selected + 1}`)
  for (const chart of charts) chart.setCursor({ left: chart.valToPos(selected / recording.header.sampleRateHz, 'x') })
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

function chart(title, fields, samples) {
  const card = document.createElement('div')
  card.className = 'chart-card'
  const heading = document.createElement('h3')
  heading.textContent = title
  const host = document.createElement('div')
  host.className = 'chart'
  card.append(heading, host)
  $('charts').append(card)
  const colors = ['#86e8ba', '#f0ba79', '#8bc7ed']
  const plot = new uPlot({
    width: Math.max(250, host.clientWidth), height: 220,
    scales: { x: { time: false } },
    axes: [{ stroke: '#8baaa8', grid: { stroke: '#274047' }, label: 'Time (s)' }, { stroke: '#8baaa8', grid: { stroke: '#274047' } }],
    series: [{}, ...fields.map(([name], i) => ({ label: name, stroke: colors[i], width: 1.5 }))],
    legend: { show: true }, cursor: { drag: { x: false, y: false } },
  }, [samples.map((_, i) => i / recording.header.sampleRateHz), ...fields.map(([, key]) => samples.map(s => s[key]))], host)
  host.addEventListener('click', event => {
    const x = plot.posToVal(event.clientX - plot.over.getBoundingClientRect().left, 'x')
    selectSample(Math.round(x * recording.header.sampleRateHz))
  })
  charts.push(plot)
}

function openRecording(buffer, name) {
  const parsed = parseRecording(buffer)
  if (playing) { clearInterval(playing); playing = null; $('playback').textContent = 'Play' }
  for (const plot of charts) plot.destroy()
  charts = []
  $('charts').replaceChildren()
  recording = parsed
  selected = null
  $('file-name').textContent = name
  $('file-summary').classList.remove('error')
  $('file-summary').textContent = `${parsed.header.count.toLocaleString()} samples · ${parsed.header.sampleRateHz} Hz · Kp ${parsed.header.kp.toFixed(3)}, Ki ${parsed.header.ki.toFixed(3)}, Kd ${parsed.header.kd.toFixed(3)} · Setpoint ${(parsed.header.setpointRad * DEG).toFixed(2)}°`
  $('sample-slider').max = Math.max(0, parsed.samples.length - 1)
  $('sample-slider').disabled = parsed.samples.length === 0
  $('playback').disabled = parsed.samples.length < 2
  $('sample-label').textContent = '—'
  if (!parsed.samples.length) return
  chart('Pitch · degrees / gyro · degrees per second', [['Pitch', 'pitchDeg'], ['Gyro', 'gyroDegS']], parsed.samples)
  chart('PID output', [['PID', 'pidOutput']], parsed.samples)
  chart('Encoder pulses', [['Motor 1', 'motor1EncoderPulses'], ['Motor 2', 'motor2EncoderPulses']], parsed.samples)
  chart('Motor PWM', [['Motor 1', 'motor1Pwm'], ['Motor 2', 'motor2Pwm']], parsed.samples)
  chart('Control interval · microseconds', [['Interval', 'controlIntervalUs']], parsed.samples)
  selectSample(0)
}
