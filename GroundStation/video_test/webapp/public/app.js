const grid = document.querySelector('#grid');
const summary = document.querySelector('#summary');
const emptyState = document.querySelector('#emptyState');
const chartsGrid = document.querySelector('#chartsGrid');
const tileTemplate = document.querySelector('#tileTemplate');
const droneModal = document.querySelector('#droneModal');
const modalCloseBtn = document.querySelector('#modalCloseBtn');
const modalTitle = document.querySelector('#modalTitle');
const modalSubtitle = document.querySelector('#modalSubtitle');
const modalTelemetry = document.querySelector('#modalTelemetry');
const modalStats = document.querySelector('#modalStats');
const modalEvent = document.querySelector('#modalEvent');
const modalVideoHost = document.querySelector('#modalVideoHost');
const players = new Map();
const chartInstances = new Map();
const chartHistory = new Map();
const telemetryRateState = new Map();
let latestState = null;
let selectedDroneName = null;
let modalMountedPlayer = null;
let modalReturnAnchor = null;

const droneColors = ['#42d392', '#6fb6ff', '#ffcc66', '#ff6b6b', '#c48cff', '#4dd0e1', '#f48fb1', '#a3e635'];
const chartDefinitions = [
  { key: 'fps', title: 'Decoded FPS', unit: 'fps' },
  { key: 'framesDropped', title: 'Dropped Frames', unit: 'frames' },
  { key: 'packetsLost', title: 'Packets Lost', unit: 'packets' },
  { key: 'jitter', title: 'Jitter', unit: 's' },
  { key: 'inboundFramesInError', title: 'MediaMTX Frame Errors', unit: 'frames' },
  { key: 'telemetryRate', title: 'Telemetry Packet Rate', unit: 'Hz' },
];

const summaryDefinitions = [
  ['detected', 'Detected Phones'],
  ['active', 'Active'],
  ['discovered', 'Discovered'],
  ['telemetry', 'Telemetry'],
  ['streams', 'Streams Ready'],
  ['browser', 'Browser WebRTC'],
  ['logFile', 'Log File', 'wide'],
];

class WhepPlayer {
  constructor(drone, tile, options = {}) {
    this.drone = drone;
    this.tile = tile;
    this.options = options;
    this.video = options.video || tile.querySelector('video');
    this.badge = options.badge || tile.querySelector('.statusBadge');
    this.events = options.events || tile.querySelector('.events');
    this.pc = null;
    this.resourceUrl = null;
    this.reconnectTimer = null;
    this.statsTimer = null;
    this.isConnecting = false;
    this.lastFramesDecoded = 0;
    this.lastStatsAt = 0;
    this.connectionLosses = 0;
    this.stalls = 0;
    this.decodeErrors = 0;
    this.lastVideoTime = 0;
    this.lastVideoProgressAt = Date.now();
    this.lastTotalVideoFrames = 0;

    this.video.addEventListener('error', () => {
      this.decodeErrors += 1;
      this.report('video_error', { error: this.video.error?.message || this.video.error?.code || 'unknown' });
    });
    this.video.addEventListener('stalled', () => this.report('video_stalled_event'));
    this.video.addEventListener('waiting', () => this.report('video_waiting'));
  }

  async connect() {
    if (this.isConnecting) return;
    this.isConnecting = true;
    this.close();
    this.setStatus('connecting', 'status-warn');
    const whepUrl = `http://${location.hostname}:8889/${encodeURIComponent(this.drone.streamName)}/whep`;

    try {
      this.pc = new RTCPeerConnection({ iceServers: [{ urls: 'stun:stun.l.google.com:19302' }] });
      this.pc.addTransceiver('video', { direction: 'recvonly' });

      this.pc.ontrack = (event) => {
        if (event.streams?.[0]) {
          this.video.srcObject = event.streams[0];
          this.video.play().catch(() => {});
          this.setStatus('playing', 'status-good');
          this.report('track_attached');
        }
      };

      this.pc.onconnectionstatechange = () => {
        const state = this.pc?.connectionState || 'closed';
        this.setStatus(state, state === 'connected' ? 'status-good' : 'status-warn');
        if (state === 'failed' || state === 'disconnected') {
          this.connectionLosses += 1;
          this.report('peer_connection_loss', { state });
          this.scheduleReconnect();
        }
      };

      const offer = await this.pc.createOffer();
      await this.pc.setLocalDescription(offer);
      await this.waitForIceGathering();

      const response = await fetch(whepUrl, {
        method: 'POST',
        headers: { 'content-type': 'application/sdp' },
        body: this.pc.localDescription.sdp,
      });

      if (!response.ok) {
        throw new Error(`WHEP ${response.status}: ${await response.text()}`);
      }

      this.resourceUrl = response.headers.get('location');
      const answerSdp = await response.text();
      await this.pc.setRemoteDescription({ type: 'answer', sdp: answerSdp });
      this.report('whep_answer_set', { url: whepUrl });
      this.startStats();
    } catch (error) {
      this.setStatus('error', 'status-bad');
      this.report('connect_error', { error: error.message });
      this.scheduleReconnect();
    } finally {
      this.isConnecting = false;
    }
  }

  waitForIceGathering() {
    if (!this.pc || this.pc.iceGatheringState === 'complete') return Promise.resolve();
    return new Promise((resolve) => {
      const timeout = setTimeout(resolve, 5000);
      this.pc.addEventListener('icegatheringstatechange', () => {
        if (this.pc?.iceGatheringState === 'complete') {
          clearTimeout(timeout);
          resolve();
        }
      });
    });
  }

  startStats() {
    clearInterval(this.statsTimer);
    this.lastFramesDecoded = 0;
    this.lastStatsAt = performance.now();
    this.lastVideoTime = this.video.currentTime || 0;
    this.lastVideoProgressAt = Date.now();
    this.statsTimer = setInterval(() => this.collectStats(), 1000);
  }

  async collectStats() {
    if (!this.pc) return;
    const now = performance.now();
    const stats = await this.pc.getStats();
    let inbound = null;
    stats.forEach((report) => {
      if (report.type === 'inbound-rtp' && report.kind === 'video') inbound = report;
    });
    if (!inbound) return;

    const seconds = Math.max((now - this.lastStatsAt) / 1000, 0.001);
    const framesDecoded = inbound.framesDecoded || 0;
    const fps = (framesDecoded - this.lastFramesDecoded) / seconds;
    this.lastFramesDecoded = framesDecoded;
    this.lastStatsAt = now;

    const currentTime = this.video.currentTime || 0;
    if (currentTime > this.lastVideoTime + 0.05) {
      this.lastVideoTime = currentTime;
      this.lastVideoProgressAt = Date.now();
    } else if (Date.now() - this.lastVideoProgressAt > 5000) {
      this.stalls += 1;
      this.lastVideoProgressAt = Date.now();
      this.report('video_progress_stalled', { stalls: this.stalls });
      this.scheduleReconnect();
    }

    const payload = {
      drone: this.drone.name,
      connectionState: this.pc.connectionState,
      iceConnectionState: this.pc.iceConnectionState,
      fps: Number(fps.toFixed(2)),
      framesDecoded,
      framesDropped: inbound.framesDropped || 0,
      packetsLost: inbound.packetsLost || 0,
      jitter: Number((inbound.jitter || 0).toFixed(4)),
      bytesReceived: inbound.bytesReceived || 0,
      frameWidth: inbound.frameWidth || this.video.videoWidth || 0,
      frameHeight: inbound.frameHeight || this.video.videoHeight || 0,
      connectionLosses: this.connectionLosses,
      stalls: this.stalls,
      decodeErrors: this.decodeErrors,
    };

    this.updateStats(payload);
    if (selectedDroneName === this.drone.name) updateModalForDrone(this.drone.name);
    addChartSample(this.drone.name, {
      fps: payload.fps,
      framesDropped: payload.framesDropped,
      packetsLost: payload.packetsLost,
      jitter: payload.jitter,
    });
    fetch('/api/client-stats', {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify(payload),
    }).catch(() => {});
  }

  updateStats(stats) {
    this.latestStats = stats;
    setText(this.tile, '[data-stat="fps"]', stats.fps.toFixed(1));
    setText(this.tile, '[data-stat="framesDecoded"]', stats.framesDecoded);
    setText(this.tile, '[data-stat="framesDropped"]', stats.framesDropped);
    setText(this.tile, '[data-stat="packetsLost"]', stats.packetsLost);
    setText(this.tile, '[data-stat="jitter"]', stats.jitter);
    setText(this.tile, '[data-stat="size"]', stats.frameWidth && stats.frameHeight ? `${stats.frameWidth}x${stats.frameHeight}` : '-');
    this.options.onStats?.(stats);
  }

  report(type, extra = {}) {
    const message = `${new Date().toLocaleTimeString()} ${type}`;
    if (this.events) this.events.textContent = message;
    if (selectedDroneName === this.drone.name) {
      modalEvent.textContent = `${message}\n${JSON.stringify(extra, null, 2)}`;
    }
    this.options.onReport?.(type, extra, message);
    fetch('/api/client-stats', {
      method: 'POST',
      headers: { 'content-type': 'application/json' },
      body: JSON.stringify({ drone: this.drone.name, event: type, ...extra }),
    }).catch(() => {});
  }

  setStatus(text, className = '') {
    if (!this.badge) return;
    this.badge.textContent = text;
    this.badge.className = `badge statusBadge ${className}`;
  }

  scheduleReconnect() {
    if (!this.drone.mediaMtx?.ready) return;
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, 2500);
  }

  close() {
    clearInterval(this.statsTimer);
    this.statsTimer = null;
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
    this.pc?.close();
    this.pc = null;
    if (this.video.srcObject) {
      this.video.srcObject.getTracks().forEach((track) => track.stop());
      this.video.srcObject = null;
    }
    if (this.video.src) {
      this.video.removeAttribute('src');
      this.video.load();
    }
  }
}

function render(state) {
  latestState = state;
  const connected = state.drones.filter((drone) => drone.telemetryConnected).length;
  const discovered = state.drones.filter((drone) => drone.lastDiscoveryAt).length;
  const readyStreams = state.drones.filter((drone) => drone.mediaMtx?.ready).length;
  const browserConnections = state.drones.filter((drone) => drone.browserStats?.connectionState === 'connected').length;
  const activeDrones = state.drones.filter((drone) => !drone.ignored).length;
  grid.dataset.count = String(Math.max(1, state.drones.length));
  grid.dataset.active = String(Math.max(1, activeDrones));
  updateSummary({
    detected: state.drones.length,
    active: activeDrones,
    discovered,
    telemetry: connected,
    streams: readyStreams,
    browser: browserConnections,
    logFile: state.logFile,
  });

  emptyState.hidden = state.drones.length > 0;

  for (const drone of state.drones) {
    let player = players.get(drone.name);
    if (!player) {
      const tile = tileTemplate.content.firstElementChild.cloneNode(true);
      tile.dataset.drone = drone.name;
      tile.tabIndex = 0;
      tile.setAttribute('role', 'button');
      tile.setAttribute('aria-label', `Open details for ${drone.name}`);
      updateText(tile.querySelector('h2'), drone.name);
      tile.querySelector('.ignoreBtn').addEventListener('click', (event) => {
        event.stopPropagation();
        setDroneIgnored(drone.name, !getDrone(drone.name)?.ignored);
      });
      tile.addEventListener('click', () => openDroneModal(drone.name));
      tile.addEventListener('keydown', (event) => {
        if (event.key === 'Enter' || event.key === ' ') {
          event.preventDefault();
          openDroneModal(drone.name);
        }
      });
      grid.appendChild(tile);
      player = new WhepPlayer(drone, tile);
      players.set(drone.name, player);
    }
    player.drone = drone;
    player.tile.classList.toggle('ignored', !!drone.ignored);
    updateText(player.tile.querySelector('.ip'), drone.ip || 'no ip');
    const ignoreButton = player.tile.querySelector('.ignoreBtn');
    updateText(ignoreButton, drone.ignored ? 'Use' : 'Ignore');
    ignoreButton.setAttribute('aria-pressed', String(!!drone.ignored));
    updateTelemetryStats(player.tile, drone);
    if (drone.ignored) {
      player.close();
      player.setStatus('ignored', 'status-warn');
    } else if (drone.mediaMtx?.ready && !player.isConnecting && player.pc?.connectionState !== 'connected') {
      player.connect();
    } else if (!drone.mediaMtx?.ready && player.pc?.connectionState !== 'connected') {
      const lostMessage = cameraLostMessage(drone);
      if (lostMessage) {
        player.setStatus('camera feed lost', 'status-bad');
        updateText(player.events, lostMessage);
      } else {
        player.setStatus(drone.status || 'waiting', drone.telemetryConnected ? 'status-warn' : 'status-bad');
      }
    }
  }

  if (selectedDroneName) updateModalForDrone(selectedDroneName);
  updateTelemetryChartSamples(state);
  updateCharts();
}

function cameraLostMessage(drone) {
  if (drone.mediaMtx?.ready || !drone.telemetryConnected) return '';
  if ((drone.telemetryPackets || 0) < 20) return '';
  return 'Device may have been idle too long and the camera feed was lost. Power-cycle the drone and let it cool down before retrying.';
}

function setText(root, selector, value) {
  updateText(root?.querySelector(selector), value);
}

function updateText(element, value) {
  if (!element) return;
  const next = String(value ?? '-');
  if (element.textContent !== next) element.textContent = next;
}

function updateClass(element, className) {
  if (element && element.className !== className) element.className = className;
}

function updateSummary(values) {
  for (const [key, label, extraClass] of summaryDefinitions) {
    let item = summary.querySelector(`[data-summary="${key}"]`);
    if (!item) {
      item = document.createElement('div');
      item.className = `summaryItem${extraClass ? ` ${extraClass}` : ''}`;
      item.dataset.summary = key;
      const labelElement = document.createElement('span');
      labelElement.textContent = label;
      const valueElement = document.createElement('strong');
      item.append(labelElement, valueElement);
      summary.appendChild(item);
    }
    updateText(item.querySelector('strong'), values[key]);
  }
}

function updateTelemetryStats(tile, drone) {
  const media = drone.mediaMtx || {};
  setText(tile, '[data-stat="telemetry"]', drone.telemetryConnected ? 'ok' : 'down');
  updateClass(tile.querySelector('[data-stat="telemetry"]'), drone.telemetryConnected ? 'status-good' : 'status-bad');
  setText(tile, '[data-stat="mediaMtx"]', media.ready ? `${media.readers?.length || 0}` : 'wait');
  updateClass(tile.querySelector('[data-stat="mediaMtx"]'), media.ready ? 'status-good' : 'status-warn');
}

function droneColor(name) {
  const index = latestState?.drones.findIndex((drone) => drone.name === name) ?? 0;
  return droneColors[Math.max(0, index) % droneColors.length];
}

function addChartSample(droneName, values) {
  const drone = getDrone(droneName);
  if (drone?.ignored) return;
  const time = Math.floor(Date.now() / 1000);
  let droneHistory = chartHistory.get(droneName);
  if (!droneHistory) {
    droneHistory = {};
    chartHistory.set(droneName, droneHistory);
  }
  for (const [key, rawValue] of Object.entries(values)) {
    const value = Number(rawValue);
    if (!Number.isFinite(value)) continue;
    const series = droneHistory[key] || [];
    const last = series[series.length - 1];
    if (last?.time === time) {
      last.value = value;
    } else {
      series.push({ time, value });
    }
    if (series.length > 600) series.splice(0, series.length - 600);
    droneHistory[key] = series;
  }
}

function updateTelemetryChartSamples(state) {
  for (const drone of state.drones) {
    if (drone.ignored) continue;
    const currentPackets = Number(drone.telemetryPackets || 0);
    const telemetry = drone.lastTelemetry || {};
    const previous = telemetryRateState.get(drone.name) || { packets: currentPackets, at: Date.now() };
    const now = Date.now();
    const seconds = Math.max((now - previous.at) / 1000, 0.001);
    const telemetryRate = Math.max(0, (currentPackets - previous.packets) / seconds);
    telemetryRateState.set(drone.name, { packets: currentPackets, at: now });
    addChartSample(drone.name, {
      telemetryRate,
      inboundFramesInError: drone.mediaMtx?.inboundFramesInError || 0,
      batteryLevel: telemetry.batteryLevel,
    });
  }
}

function createSeries(chart, options) {
  if (chart.addSeries && window.LightweightCharts?.LineSeries) {
    return chart.addSeries(window.LightweightCharts.LineSeries, options);
  }
  return chart.addLineSeries(options);
}

function ensureCharts() {
  if (!chartsGrid) return false;
  if (!window.LightweightCharts) {
    chartsGrid.innerHTML = '<div class="emptyState">Charts are unavailable because Lightweight Charts did not load. Check network access to the CDN.</div>';
    return false;
  }
  for (const definition of chartDefinitions) {
    if (chartInstances.has(definition.key)) continue;
    const card = document.createElement('section');
    card.className = 'chartCard';
    card.innerHTML = `<div class="chartTitle"><h3>${definition.title}</h3><span>${definition.unit}</span></div><div class="chartCanvas"></div>`;
    chartsGrid.appendChild(card);
    const canvas = card.querySelector('.chartCanvas');
    const chart = window.LightweightCharts.createChart(canvas, {
      autoSize: true,
      layout: { background: { color: '#1a1d20' }, textColor: '#edf1f4' },
      grid: { vertLines: { color: '#2b3137' }, horzLines: { color: '#2b3137' } },
      timeScale: { timeVisible: true, secondsVisible: true, borderColor: '#343b42' },
      rightPriceScale: { borderColor: '#343b42' },
      crosshair: { mode: 1 },
    });
    chartInstances.set(definition.key, { chart, card, series: new Map() });
  }
  return true;
}

function updateCharts() {
  const chartsPanel = document.querySelector('[data-panel="charts"]');
  if (chartsPanel && !chartsPanel.classList.contains('active')) return;
  if (!ensureCharts() || !latestState) return;
  const activeDrones = latestState.drones.filter((drone) => !drone.ignored);
  for (const definition of chartDefinitions) {
    const instance = chartInstances.get(definition.key);
    if (!instance) continue;
    for (const drone of activeDrones) {
      let series = instance.series.get(drone.name);
      if (!series) {
        series = createSeries(instance.chart, {
          title: drone.name,
          color: droneColor(drone.name),
          lineWidth: 2,
          lastValueVisible: true,
          priceLineVisible: false,
        });
        instance.series.set(drone.name, series);
      }
      const data = chartHistory.get(drone.name)?.[definition.key] || [];
      series.setData(data);
    }
    for (const [name, series] of instance.series.entries()) {
      if (!activeDrones.some((drone) => drone.name === name)) {
        instance.chart.removeSeries(series);
        instance.series.delete(name);
      }
    }
  }
}

async function setDroneIgnored(name, ignored) {
  const player = players.get(name);
  if (ignored) player?.close();
  await fetch(`/api/drones/${encodeURIComponent(name)}/ignore`, {
    method: 'POST',
    headers: { 'content-type': 'application/json' },
    body: JSON.stringify({ ignored }),
  }).catch(() => {});
}

function getDrone(name) {
  return latestState?.drones.find((drone) => drone.name === name) || null;
}

function updateDetailList(root, rows) {
  for (const [key, label, value, className = ''] of rows) {
    let row = root.querySelector(`[data-detail="${key}"]`);
    if (!row) {
      row = document.createElement('div');
      row.dataset.detail = key;
      const term = document.createElement('dt');
      term.textContent = label;
      const description = document.createElement('dd');
      row.append(term, description);
      root.appendChild(row);
    }
    const description = row.querySelector('dd');
    updateText(description, value);
    updateClass(description, className);
  }
}

function openModalDetails() {
  droneModal.querySelectorAll('details').forEach((details) => {
    details.open = true;
  });
}

function restoreModalVideo() {
  if (!modalMountedPlayer || !modalReturnAnchor) return;
  modalMountedPlayer.tile.insertBefore(modalMountedPlayer.videoWrap, modalReturnAnchor);
  modalReturnAnchor.remove();
  modalMountedPlayer = null;
  modalReturnAnchor = null;
}

function mountPlayerVideo(player) {
  if (modalMountedPlayer === player) return;
  restoreModalVideo();
  const videoWrap = player.tile.querySelector('.videoWrap');
  if (!videoWrap) return;
  modalReturnAnchor = document.createComment('video-return-anchor');
  player.tile.insertBefore(modalReturnAnchor, videoWrap);
  player.videoWrap = videoWrap;
  modalVideoHost.replaceChildren(videoWrap);
  modalMountedPlayer = player;
}

function updateModalForDrone(name) {
  const drone = getDrone(name);
  if (!drone) return;
  const telemetry = drone.lastTelemetry || {};
  const media = drone.mediaMtx || {};
  const browserStats = modalMountedPlayer?.latestStats || drone.browserStats || {};
  const telemetryAge = drone.lastTelemetryAt ? `${Math.max(0, Math.round((Date.now() - Date.parse(drone.lastTelemetryAt)) / 1000))}s ago` : 'none';
  const lostMessage = cameraLostMessage(drone);

  updateText(modalTitle, drone.name);
  updateText(modalSubtitle, `${drone.ip || 'no ip'} | ${drone.ignored ? 'ignored' : 'active'} | telemetry ${drone.telemetryConnected ? 'ok' : 'down'} | stream ${media.ready ? 'ready' : 'waiting'}`);
  updateDetailList(modalTelemetry, [
    ['telemetry', 'Telemetry', drone.telemetryConnected ? `ok (${telemetryAge})` : `down (${telemetryAge})`, drone.telemetryConnected ? 'status-good' : 'status-bad'],
    ['packets', 'Packets', drone.telemetryPackets],
    ['battery', 'Battery', telemetry.batteryLevel === undefined ? '-' : `${telemetry.batteryLevel}%`],
    ['flightMode', 'Flight Mode', telemetry.flightMode],
    ['location', 'Location', telemetry.location ? `${telemetry.location.latitude}, ${telemetry.location.longitude}, ${telemetry.location.altitude}m` : '-'],
    ['phoneBattery', 'Phone Battery', telemetry.phoneLocation?.battery === undefined ? '-' : `${telemetry.phoneLocation.battery}%`],
    ['wifiRssi', 'Wi-Fi RSSI', telemetry.phoneLocation?.wifiRssi],
    ['lastError', 'Last Error', drone.lastError || '-'],
  ]);
  updateDetailList(modalStats, [
    ['mediamtx', 'MediaMTX', media.ready ? `ready (${media.readers?.length || 0} readers)` : 'waiting', media.ready ? 'status-good' : 'status-warn'],
    ['operatorMessage', 'Operator Message', lostMessage || '-'],
    ['codec', 'Codec', media.tracks?.join(', ') || '-'],
    ['inboundErrors', 'Inbound Errors', media.inboundFramesInError ?? '-'],
    ['fps', 'FPS', browserStats.fps === undefined ? '-' : Number(browserStats.fps).toFixed(1)],
    ['decodedFrames', 'Decoded Frames', browserStats.framesDecoded],
    ['droppedFrames', 'Dropped Frames', browserStats.framesDropped],
    ['packetsLost', 'Packets Lost', browserStats.packetsLost],
    ['jitter', 'Jitter', browserStats.jitter],
    ['connectionLosses', 'Connection Losses', browserStats.connectionLosses],
    ['stalls', 'Stalls', browserStats.stalls],
    ['decodeErrors', 'Decode Errors', browserStats.decodeErrors],
  ]);

  if (drone.ignored) {
    modalMountedPlayer?.setStatus('ignored', 'status-warn');
  } else if (lostMessage) {
    modalMountedPlayer?.setStatus('camera feed lost', 'status-bad');
    modalEvent.textContent = lostMessage;
  } else if (!media.ready && modalMountedPlayer?.pc?.connectionState !== 'connected') {
    modalMountedPlayer?.setStatus(drone.telemetryConnected ? 'waiting for stream' : 'waiting for telemetry', drone.telemetryConnected ? 'status-warn' : 'status-bad');
  }
}

function openDroneModal(name) {
  const drone = getDrone(name);
  const player = players.get(name);
  if (!drone || !player) return;
  selectedDroneName = name;
  mountPlayerVideo(player);
  openModalDetails();
  updateModalForDrone(name);
  if (!droneModal.open) droneModal.showModal();
}

function closeDroneModal() {
  selectedDroneName = null;
  restoreModalVideo();
  if (droneModal.open) droneModal.close();
}

async function loadState() {
  const response = await fetch('/api/drones');
  render(await response.json());
}

function reconnectAll() {
  for (const player of players.values()) {
    if (!player.drone.ignored && player.drone.mediaMtx?.ready) player.connect();
  }
}

document.querySelector('#discoverBtn').addEventListener('click', () => {
  fetch('/api/discover', { method: 'POST' }).catch(() => {});
});
document.querySelector('#reconnectBtn').addEventListener('click', reconnectAll);
modalCloseBtn.addEventListener('click', closeDroneModal);
droneModal.addEventListener('close', closeDroneModal);
droneModal.addEventListener('click', (event) => {
  if (event.target === droneModal) closeDroneModal();
});

document.querySelectorAll('.tabButton').forEach((button) => {
  button.addEventListener('click', () => {
    const tab = button.dataset.tab;
    document.querySelectorAll('.tabButton').forEach((item) => item.classList.toggle('active', item === button));
    document.querySelectorAll('.tabPanel').forEach((panel) => panel.classList.toggle('active', panel.dataset.panel === tab));
    if (tab === 'charts') {
      updateCharts();
      setTimeout(updateCharts, 50);
    }
  });
});

const events = new EventSource('/api/events');
events.onmessage = (event) => {
  const message = JSON.parse(event.data);
  if (message.type === 'state') render(message.payload);
};

loadState();
setInterval(() => {
  if (latestState) render(latestState);
}, 2000);
