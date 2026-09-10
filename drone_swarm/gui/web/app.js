import { SwarmScene } from './scene.js';

const $ = id => document.getElementById(id);
const numberSettings = ['count', 'size', 'altitude', 'speed', 'transition_time', 'min_distance', 'brightness'];
const title = value => String(value || '').toLowerCase().replaceAll('_', ' ').replace(/\b\w/g, letter => letter.toUpperCase());
const fixed = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
let snapshot = null;
let available = false;
let busy = false;
let pendingCommands = 0;
let settingsDirty = false;
let formationDirty = false;
let formationAwaitingAck = null;
let formationNotice = null;
let commandSequence = 0;
let cameraInitialized = false;
let configSyncAfter = 0;
let sourceData = null;
let previewKey = null;
let pollTimer = null;
let noticeTimer = null;
let view = null;
let graphicsError = null;

try {
  view = new SwarmScene($('stage'), fps => { $('render-rate').textContent = fixed(fps, 0); });
} catch (error) {
  graphicsError = 'The 3D renderer could not start. Enable WebGL or reopen the desktop app.';
  console.error('AEROS renderer:', error);
}

function notify(message, error = false) {
  clearTimeout(noticeTimer);
  $('notice-text').textContent = message;
  $('notice').classList.toggle('error', error);
  $('notice').hidden = false;
  if (!error) noticeTimer = setTimeout(() => { $('notice').hidden = true; }, 6000);
}

function formatTime(value) {
  if (!Number.isFinite(value)) return '—';
  const seconds = Math.max(0, Math.floor(value));
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor(seconds / 60) % 60;
  const prefix = hours ? `${String(hours).padStart(2, '0')}:` : '';
  return `${prefix}${String(minutes).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
}

function rgbToHex(values) {
  return '#' + values.map(value => Math.round(Math.max(0, Math.min(1, value)) * 255).toString(16).padStart(2, '0')).join('');
}

function imageGeometryKey(config) {
  const values = [config?.count, config?.size, config?.altitude];
  return values.every(Number.isFinite) ? JSON.stringify(values) : null;
}

function invalidateOutdatedPreview(config) {
  const currentKey = imageGeometryKey(config);
  if (!previewKey || !currentKey || previewKey === currentKey) return;
  previewKey = null;
  $('points-preview').hidden = true;
  $('points-preview').removeAttribute('src');
  $('points-empty').hidden = false;
  $('image-metadata').textContent = 'Reprocess to preview current settings';
}

function syncSettings(config) {
  if (!config || settingsDirty || performance.now() < configSyncAfter) return;
  for (const key of numberSettings) {
    if (Number.isFinite(config[key]) && document.activeElement !== $(key)) $(key).value = config[key];
  }
  $('collision_avoidance').checked = Boolean(config.collision_avoidance);
  if (config.color_mode) $('color_mode').value = config.color_mode;
  if (Array.isArray(config.color) && config.color.length === 3) $('color').value = rgbToHex(config.color);
  $('brightness-output').textContent = `${Math.round(Number($('brightness').value) * 100)}%`;
}

function updateButtons() {
  const connected = available && snapshot?.connected;
  const state = snapshot?.state || 'CONNECTING';
  const stopped = ['IDLE', 'STOPPED'].includes(state);
  for (const button of document.querySelectorAll('[data-command]')) button.disabled = busy || !connected;
  for (const id of ['upload-image', 'choose-image', 'image-file']) $(id).disabled = busy;
  $('count').disabled = !stopped || busy;
  $('min_distance').disabled = !stopped || busy;
  $('stop-hint').textContent = stopped ? 'Count and separation can be changed while stopped.' : 'Stop the show to change count or separation.';
  $('process-image').disabled = busy || !sourceData || !connected || Boolean(snapshot?.pending_config);
  $('start-show').disabled = busy || !connected || !['IDLE', 'STOPPED', 'FORMATION_COMPLETE', 'ANIMATING'].includes(state);
  $('pause-show').disabled = busy || !connected || !['TAKEOFF', 'TRANSITIONING', 'PAUSED', 'ANIMATING', 'FORMATION_COMPLETE'].includes(state);
  $('pause-label').textContent = state === 'PAUSED' ? 'Resume' : 'Pause';
  $('pause-symbol').textContent = state === 'PAUSED' ? '▶' : 'Ⅱ';
  // Stop can interrupt a show even while an image worker command is pending.
  $('stop-show').disabled = !available;
  $('reset-show').disabled = busy || !connected;
  $('settings-dirty').hidden = !settingsDirty && !snapshot?.pending_config;
  $('settings-dirty').textContent = settingsDirty ? 'UNSAVED' : 'QUEUED';
}

function showConnection() {
  const connected = available && Boolean(snapshot?.connected);
  const isError = snapshot?.state === 'ERROR';
  $('connection').className = `connection ${connected ? 'online' : isError ? 'error' : 'waiting'}`;
  $('connection').querySelector('span').textContent = connected ? 'Gazebo connected' : available ? 'Waiting for Gazebo' : 'Workspace disconnected';
  const hasPositions = connected && Array.isArray(snapshot?.positions) && snapshot.positions.length > 0;
  $('stage-message').hidden = hasPositions && !graphicsError;
  $('stage-message-title').textContent = graphicsError ? '3D view unavailable' : available ? 'Waiting for measured positions' : 'Connecting to your sky';
  $('stage-message-text').textContent = graphicsError || (available ?
    (snapshot?.error || 'Gazebo and the ROS bridge must be connected to display the fleet.') :
    'The local workspace is unreachable. Start the simulator; this page will reconnect automatically.');
  $('stage-subtitle').textContent = hasPositions ? 'Measured Gazebo poses · world frame' : 'Waiting for measured drone positions';
  view?.update(snapshot?.positions, snapshot?.colors, connected);
}

function renderState(state) {
  snapshot = state;
  available = true;
  if (formationNotice && state.last_command?.request_id === formationNotice.requestId) {
    if (state.last_command.accepted) {
      const name = formationNotice.formation;
      notify(state.pending_formation === name ?
        `${title(name)} formation queued for the next formation boundary.` :
        `${title(name)} formation applied.`);
    }
    formationNotice = null;
  }
  const connected = Boolean(state.connected);
  syncSettings(state.pending_config || state.config);
  invalidateOutdatedPreview(state.config);
  const selectedFormation = state.pending_formation || state.formation;
  if (formationAwaitingAck === selectedFormation) formationAwaitingAck = null;
  if (!formationDirty && !formationAwaitingAck && selectedFormation && $('formation').querySelector(`option[value="${CSS.escape(selectedFormation)}"]`)) {
    $('formation').value = selectedFormation;
  }
  if (connected && view && !cameraInitialized) {
    view.fitConfig(state.config);
    cameraInitialized = true;
  }
  $('formation-title').textContent = state.formation ? `${title(state.formation)} formation` : 'Your sky, reimagined.';
  const actualCount = connected ? state.actual_count ?? state.positions?.length : null;
  $('actual-count').textContent = Number.isFinite(actualCount) ? actualCount : '—';
  $('capacity-label').textContent = `/ ${state.capacity ?? state.config?.count ?? '—'}`;
  $('fleet-status').textContent = connected ? `${state.config?.count ?? actualCount} drones in this show` : 'Awaiting fresh feedback';
  $('simulation-state').textContent = String(state.state || 'CONNECTING').replaceAll('_', ' ');
  $('simulation-state').classList.toggle('error', state.state === 'ERROR');
  const measuredTime = state.actual_simulation_time ?? state.simulation_time;
  $('simulation-time').textContent = connected ? formatTime(measuredTime) : '—';
  $('actual-speed').textContent = connected ? fixed(state.actual_speed_m_s ?? state.speed_m_s) : '—';
  $('actual-separation').textContent = connected ? fixed(state.actual_min_separation_m ?? state.min_separation_m) : '—';
  $('tracking-error').textContent = connected ? fixed(state.tracking_error_m, 3) : '—';
  $('control-rate').textContent = connected ? fixed(state.control_update_hz, 1) : '—';
  $('real-time-factor').textContent = connected ? fixed(state.real_time_factor) : '—';
  $('pose-age').textContent = Number.isFinite(state.feedback_age_s) ? `${fixed(state.feedback_age_s * 1000, 0)} ms` : '—';
  $('depth-layers').textContent = Number.isFinite(state.depth_layers) ? state.depth_layers : '—';
  $('telemetry-message').textContent = state.error || state.message || 'Measured state from the Gazebo fleet backend.';
  $('transport-caption').textContent = busy ? 'Sending command…' : state.message || title(state.state) || 'Ready when you are';
  const progress = Number.isFinite(state.progress) ? Math.max(0, Math.min(1, state.progress)) : 0;
  $('progress').value = progress;
  $('progress-percent').textContent = `${Math.round(progress * 100)}%`;
  $('progress-label').textContent = state.state === 'TAKEOFF' ? 'Takeoff progress' : 'Formation progress';
  showConnection();
  updateButtons();
}

async function poll() {
  const started = performance.now();
  try {
    const response = await fetch('/api/state', { cache: 'no-store', signal: AbortSignal.timeout(2000) });
    if (!response.ok) throw new Error(`Workspace returned HTTP ${response.status}`);
    const data = await response.json();
    if (!data || typeof data !== 'object' || Array.isArray(data)) throw new Error('Invalid state response');
    renderState(data);
  } catch (error) {
    available = false;
    showConnection();
    updateButtons();
    for (const id of ['actual-count', 'simulation-time', 'actual-speed', 'actual-separation', 'tracking-error', 'control-rate', 'real-time-factor', 'pose-age']) $(id).textContent = '—';
    $('simulation-state').textContent = 'DISCONNECTED';
    $('fleet-status').textContent = 'Awaiting feedback';
    $('telemetry-message').textContent = 'Connection lost. Measured drone positions are hidden until fresh feedback returns.';
  } finally {
    pollTimer = setTimeout(poll, Math.max(25, 100 - (performance.now() - started)));
  }
}

async function send(command, successMessage) {
  if (busy && command.action !== 'stop') return null;
  pendingCommands++;
  busy = true;
  updateButtons();
  $('transport-caption').textContent = 'Sending command…';
  try {
    const response = await fetch('/api/command', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(command), signal: AbortSignal.timeout(20000),
    });
    const result = await response.json();
    if (!response.ok || result.ok === false || result.error) throw new Error(result.error || `Command failed (HTTP ${response.status})`);
    if (successMessage) notify(successMessage);
    return result;
  } catch (error) {
    notify(error.name === 'TimeoutError' ? 'The controller did not respond in time. Check Gazebo and the ROS bridge.' : error.message, true);
    return null;
  } finally {
    pendingCommands--;
    busy = pendingCommands > 0;
    updateButtons();
  }
}

function readSettings() {
  if (!$('settings-form').reportValidity()) return null;
  const settings = Object.fromEntries(numberSettings.map(key => [key, Number($(key).value)]));
  const hex = $('color').value;
  settings.color = [1, 3, 5].map(index => parseInt(hex.slice(index, index + 2), 16) / 255);
  settings.color_mode = $('color_mode').value;
  settings.collision_avoidance = $('collision_avoidance').checked;
  return settings;
}

function fileAsDataURL(blob) {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onload = () => resolve(reader.result);
    reader.onerror = () => reject(new Error('The image file could not be read.'));
    reader.readAsDataURL(blob);
  });
}

async function selectImage(blob, name) {
  if (busy) throw new Error('Wait for the current command to finish before choosing another image.');
  if (blob.size > 8 * 1024 * 1024) throw new Error('Choose a PNG or JPEG smaller than 8 MiB.');
  if (!['image/png', 'image/jpeg'].includes(blob.type)) throw new Error('Choose a PNG or JPEG image.');
  const dataURL = await fileAsDataURL(blob);
  if (busy) throw new Error('Wait for the current command to finish before choosing another image.');
  sourceData = dataURL.slice(dataURL.indexOf(',') + 1);
  previewKey = null;
  $('source-preview').src = dataURL;
  $('source-preview').hidden = false;
  $('source-empty').hidden = true;
  $('points-preview').hidden = true;
  $('points-preview').removeAttribute('src');
  $('points-empty').hidden = false;
  $('image-name').textContent = name;
  $('image-metadata').textContent = 'Image ready. Process it with the current drone count and formation size.';
  updateButtons();
}

async function processImage() {
  if (!sourceData || busy) return;
  if (settingsDirty) {
    notify('Apply your flight settings before processing the image.', true);
    return;
  }
  if (snapshot?.pending_config) { notify('Settings are queued. Wait for the transition to finish before processing an image.', true); return; }
  if (!$('image-threshold').reportValidity()) return;
  const processingKey = imageGeometryKey(snapshot?.config);
  const result = await send({ action: 'image', data: sourceData,
    mode: $('image-mode').value, threshold: Number($('image-threshold').value), invert: $('image-invert').checked },
  'Image processed. Apply the Image formation to use these points.');
  if (!result) return;
  previewKey = processingKey;
  if (typeof result.preview === 'string' && result.preview) {
    $('points-preview').src = `data:image/png;base64,${result.preview}`;
    $('points-preview').hidden = false;
    $('points-empty').hidden = true;
  }
  const metadata = result.image || {};
  const dimensions = Array.isArray(metadata.resized_dimensions) ? metadata.resized_dimensions.join(' × ') : '';
  $('image-metadata').textContent = `${metadata.count ?? snapshot?.config?.count ?? '—'} drone points${dimensions ? ` · ${dimensions} px` : ''}${Number.isFinite(metadata.candidates) ? ` · ${metadata.candidates.toLocaleString()} candidates` : ''}`;
  invalidateOutdatedPreview(snapshot?.config);
  $('formation').value = 'image';
  formationDirty = true;
}

$('settings-form').addEventListener('input', () => {
  settingsDirty = true;
  $('brightness-output').textContent = `${Math.round(Number($('brightness').value) * 100)}%`;
  updateButtons();
});
$('settings-form').addEventListener('submit', async event => {
  event.preventDefault();
  const settings = readSettings();
  if (!settings) return;
  const result = await send({ action: 'configure', settings });
  if (result) {
    notify(result.pending_config ?
      'Motion settings queued for the next formation boundary. Light settings applied.' :
      'Flight and light settings applied.');
    settingsDirty = false;
    configSyncAfter = performance.now() + 700;
    updateButtons();
  }
});
$('formation').addEventListener('change', () => { formationDirty = true; });
$('apply-formation').addEventListener('click', async () => {
  if (settingsDirty) { notify('Apply your flight settings before applying a formation.', true); return; }
  const formation = $('formation').value;
  const requestId = `formation-${Date.now()}-${++commandSequence}`;
  formationNotice = { formation, requestId };
  const result = await send({ action: 'apply', formation, request_id: requestId });
  if (!result && formationNotice?.requestId === requestId) formationNotice = null;
  if (result) { formationDirty = false; formationAwaitingAck = formation; view?.fitConfig(snapshot?.pending_config || snapshot?.config); }
});
$('start-show').addEventListener('click', async () => {
  if (settingsDirty) { notify('Apply your settings before starting the show.', true); return; }
  if (formationDirty) { notify('Apply your selected formation before starting the show.', true); return; }
  await send({ action: 'start' }, 'Show started.');
});
$('pause-show').addEventListener('click', () => send({ action: snapshot?.state === 'PAUSED' ? 'resume' : 'pause' }));
$('stop-show').addEventListener('click', () => send({ action: 'stop' }, 'Show stopped at the current positions.'));
$('reset-show').addEventListener('click', async () => {
  const result = await send({ action: 'reset' }, 'The fleet has been reset to its launch grid.');
  if (result) { settingsDirty = false; formationDirty = false; formationAwaitingAck = null; view?.fitConfig(snapshot?.config); }
});
for (const id of ['upload-image', 'choose-image']) $(id).addEventListener('click', () => {
  if (!busy) $('image-file').click();
});
$('image-file').addEventListener('change', async event => {
  const file = event.target.files[0];
  if (!file) return;
  try { await selectImage(file, file.name); } catch (error) { notify(error.message, true); }
  event.target.value = '';
});
$('upload-image').addEventListener('dragover', event => { event.preventDefault(); });
$('upload-image').addEventListener('drop', async event => {
  event.preventDefault();
  if (busy) { notify('Wait for the current command to finish before choosing another image.', true); return; }
  const file = event.dataTransfer.files[0];
  if (!file) return;
  try { await selectImage(file, file.name); } catch (error) { notify(error.message, true); }
});
$('use-sample').addEventListener('click', async () => {
  if (busy) return;
  try {
    const response = await fetch('/api/sample', { signal: AbortSignal.timeout(5000) });
    if (!response.ok) throw new Error('The sample image is unavailable.');
    await selectImage(await response.blob(), 'AEROS · Starlight sample.png');
    await processImage();
  } catch (error) { notify(error.message, true); }
});
$('process-image').addEventListener('click', processImage);
$('dismiss-notice').addEventListener('click', () => { $('notice').hidden = true; });
for (const mode of ['fit', 'front', 'top']) $(`view-${mode}`).addEventListener('click', () => {
  view?.setView(mode);
  for (const button of document.querySelectorAll('.view-controls button')) button.classList.remove('selected');
  $(`view-${mode}`).classList.add('selected');
});
window.addEventListener('beforeunload', () => {
  clearTimeout(pollTimer);
  clearTimeout(noticeTimer);
  view?.dispose();
});
updateButtons();
poll();
