const grid = document.querySelector('#grid');
const summary = document.querySelector('#summary');
const emptyState = document.querySelector('#emptyState');
const healthGrid = document.querySelector('#healthGrid');
const telemetryGrid = document.querySelector('#telemetryGrid');
const videoChartsGrid = document.querySelector('#videoChartsGrid');
const telemetryChartsGrid = document.querySelector('#telemetryChartsGrid');
const publishTargets = document.querySelector('#publishTargets');
const publishGrid = document.querySelector('#publishGrid');
const tileTemplate = document.querySelector('#tileTemplate');
const droneModal = document.querySelector('#droneModal');
const modalCloseBtn = document.querySelector('#modalCloseBtn');
const modalTitle = document.querySelector('#modalTitle');
const modalSubtitle = document.querySelector('#modalSubtitle');
const modalTelemetry = document.querySelector('#modalTelemetry');
const modalStats = document.querySelector('#modalStats');
const modalRecovery = document.querySelector('#modalRecovery');
const modalEvent = document.querySelector('#modalEvent');
const modalVideoHost = document.querySelector('#modalVideoHost');

const players = new Map();
const healthCards = new Map();
const telemetryCards = new Map();
const chartInstances = new Map();
const chartHistory = new Map();
const telemetryRateState = new Map();
const healthTrendState = new Map();

let latestState = null;
let selectedDroneName = null;
let modalMountedPlayer = null;
let modalReturnAnchor = null;

const droneColors = ['#42d392', '#6fb6ff', '#ffcc66', '#ff6b6b', '#c48cff', '#4dd0e1', '#f48fb1', '#a3e635'];
const receiverBufferPolicy = {
  mode: 'adaptive-low-latency',
  minMs: 40,
  initialMs: 60,
  stepMs: 20,
  maxMs: 220,
  stableAfterMs: 12_000,
};

const videoChartDefinitions = [
  { key: 'fps', title: 'Decoded FPS', unit: 'fps' },
  { key: 'bitrateKbps', title: 'Browser Receive Bitrate', unit: 'kbps' },
  { key: 'framesDropped', title: 'Dropped Frames', unit: 'frames' },
  { key: 'packetsLost', title: 'Packets Lost', unit: 'packets' },
  { key: 'jitter', title: 'Jitter', unit: 's' },
  { key: 'jitterBufferTargetMs', title: 'Receiver Jitter Target', unit: 'ms' },
  { key: 'jitterBufferDelayMs', title: 'Receiver Jitter Delay', unit: 'ms/frame' },
  { key: 'freezeCount', title: 'Receiver Freezes', unit: 'count' },
  { key: 'nackCount', title: 'NACK Requests', unit: 'count' },
  { key: 'pliCount', title: 'PLI Requests', unit: 'count' },
  { key: 'keyFramesDecoded', title: 'Keyframes Decoded', unit: 'frames' },
  { key: 'inboundFramesInError', title: 'MediaMTX Frame Errors', unit: 'frames' },
];

const telemetryChartDefinitions = [
  { key: 'telemetryRate', title: 'Telemetry Packet Rate', unit: 'Hz' },
  { key: 'batteryLevel', title: 'Battery Level', unit: '%' },
  { key: 'satelliteCount', title: 'Satellite Count', unit: 'sat' },
  { key: 'altitude', title: 'Altitude ASL', unit: 'm' },
  { key: 'distanceToHome', title: 'Distance To Home', unit: 'm' },
  { key: 'speedMagnitude', title: 'Speed Magnitude', unit: 'm/s' },
  { key: 'phoneBattery', title: 'Phone Battery', unit: '%' },
  { key: 'wifiRssi', title: 'Wi-Fi RSSI', unit: 'dBm' },
];

const chartGroups = {
  videoCharts: { grid: videoChartsGrid, definitions: videoChartDefinitions },
  telemetryCharts: { grid: telemetryChartsGrid, definitions: telemetryChartDefinitions },
};

const summaryDefinitions = [
  ['detected', 'Detected Phones'],
  ['active', 'Active'],
  ['discovered', 'Discovered'],
  ['telemetry', 'Telemetry'],
  ['streams', 'Streams Ready'],
  ['browser', 'Browser WebRTC'],
  ['logFile', 'Log File', 'wide'],
];

const publishCatalog = [
  ['HTTP Flight Control', 'Direct phone commands on http://DRONE_IP:8080. These hit the WildBridge Android app first; the phone then calls the DJI SDK. Use these for quick bench tests or when ROS is not running.', [
    ['Takeoff', 'POST', '/send/takeoff', '', 'Requests DJI takeoff. The aircraft arms as part of the DJI takeoff flow.'],
    ['Land', 'POST', '/send/land', '', 'Requests immediate landing from the current position.'],
    ['Return To Home', 'POST', '/send/RTH', '', 'Starts DJI return-to-home using the currently configured home point and RTH altitude.'],
    ['Goto Altitude', 'POST', '/send/gotoAltitude', '35', 'Commands a target altitude in meters while keeping the current horizontal position.'],
    ['Goto Yaw', 'POST', '/send/gotoYaw', '180', 'Commands an absolute aircraft heading in degrees.'],
    ['Set RTH Altitude', 'POST', '/send/setRTHAltitude', '60', 'Updates the DJI return-to-home altitude in meters before an RTH command is needed.'],
  ]],
  ['HTTP Virtual Stick', 'Manual-control style inputs and interruption commands. Virtual stick must be enabled before velocity-like control is useful, and stick values are normalized rather than physical units.', [
    ['Enable Virtual Stick', 'POST', '/send/enableVirtualStick', '', 'Enables DJI virtual stick mode so subsequent stick commands are accepted.'],
    ['Stick Command', 'POST', '/send/stick', '0.0,0.2,0.0,-0.1', 'Sends leftX,leftY,rightX,rightY normalized stick values. Typical range is -1.0 to 1.0.'],
    ['Abort Mission', 'POST', '/send/abortMission', '', 'Stops the current mission flow, zeros stick input, and disables virtual stick where supported.'],
    ['Abort All', 'POST', '/send/abortAll', '', 'Broad cancellation path for queued or active mission activity.'],
    ['Deactivate Manual Override', 'POST', '/send/deactivateManualOverride', '', 'Returns control to autonomous command handling after manual override has been active.'],
  ]],
  ['HTTP Camera And Gimbal', 'Payload commands routed through the phone to DJI camera/gimbal APIs. These are useful for operator payload checks independently of navigation.', [
    ['Gimbal Pitch Move', 'POST', '/send/gimbal/pitch', '0,-20,0', 'Sends an absolute roll,pitch,yaw command; this route is normally used for pitch.'],
    ['Gimbal Yaw Move', 'POST', '/send/gimbal/yaw', '0,0,45', 'Sends an absolute roll,pitch,yaw command; this route is normally used for yaw.'],
    ['Camera Zoom', 'POST', '/send/camera/zoom', '2.0', 'Sets the camera zoom ratio directly, subject to the aircraft/camera zoom limits.'],
    ['Start Recording', 'POST', '/send/camera/startRecording', '', 'Starts camera recording on the aircraft payload.'],
    ['Stop Recording', 'POST', '/send/camera/stopRecording', '', 'Stops camera recording on the aircraft payload.'],
  ]],
  ['HTTP Waypoint And Mission', 'Higher-level navigation routes for single-waypoint and multi-waypoint movement. These commands carry coordinates in the request body, so check units before publishing from scripts.', [
    ['Goto Waypoint', 'POST', '/send/gotoWP', '55.6761,12.5683,35', 'Single lat,lon,alt waypoint command in decimal degrees and meters.'],
    ['Goto Waypoint With PID', 'POST', '/send/gotoWPwithPID', '55.6761,12.5683,35,90,4', 'Waypoint plus yaw and maxSpeed: lat,lon,alt,yaw,maxSpeed.'],
    ['Navigate Trajectory', 'POST', '/send/navigateTrajectory', '55.6761,12.5683,25;55.6764,12.5688,30;55.6768,12.5692,32,90', 'Semicolon-separated waypoint list; final waypoint may include yaw.'],
    ['Navigate DJI Native', 'POST', '/send/navigateTrajectoryDJINative', '4.0;55.6761,12.5683,25;55.6768,12.5692,32', 'DJI native mission upload with speed first, then at least two lat,lon,alt waypoints.'],
    ['Abort DJI Mission', 'POST', '/send/abort/DJIMission', '', 'Stops the active DJI native mission path.'],
  ]],
  ['HTTP Sensing And Status', 'Read/control endpoints for operator state, AutoSensing, and phone-side configuration. These are useful for checking what the app believes before sending motion commands.', [
    ['Start AutoSensing', 'POST', '/send/autoSensing/start', '', 'Enables AutoSensing on the device.'],
    ['Stop AutoSensing', 'POST', '/send/autoSensing/stop', '', 'Disables AutoSensing on the device.'],
    ['Manual Override State', 'POST', '/get/isManualOverrideActive', '', 'Returns whether manual override is currently active.'],
    ['AutoSensing Status', 'POST', '/get/autoSensing/status', '', 'Returns AutoSensing enablement and target count.'],
    ['AutoSensing Targets', 'POST', '/get/autoSensing/targets', '', 'Returns the current detected target array from the phone app.'],
    ['Config', 'GET', '/config', '', 'Reads IP, drone name, and port configuration from the phone app.'],
  ]],
];

const rosPublishCatalog = [
  ['ROS Services', 'ROS 2 service calls exposed by wildbridge_mavros. These call the bridge node on the ground station, which then forwards to the phone HTTP/DJI path.', [
    ['Takeoff', 'service', '/mavros/cmd/takeoff', 'std_srvs/srv/Trigger', 'ros2 service call /DRONE_NS/mavros/cmd/takeoff std_srvs/srv/Trigger {}', 'Requests DJI takeoff through the MAVROS-compatible bridge.'],
    ['Land', 'service', '/mavros/cmd/land', 'std_srvs/srv/Trigger', 'ros2 service call /DRONE_NS/mavros/cmd/land std_srvs/srv/Trigger {}', 'Requests landing through the bridge.'],
    ['Return To Launch', 'service', '/mavros/cmd/rtl', 'std_srvs/srv/Trigger', 'ros2 service call /DRONE_NS/mavros/cmd/rtl std_srvs/srv/Trigger {}', 'Requests DJI return-to-home through a MAVROS-style RTL route.'],
    ['Arming Compatibility', 'service', '/mavros/cmd/arming', 'std_srvs/srv/SetBool', 'ros2 service call /DRONE_NS/mavros/cmd/arming std_srvs/srv/SetBool "{data: true}"', 'Compatibility route. DJI aircraft effectively arm on takeoff; false maps to landing.'],
    ['Enable Offboard', 'service', '/mavros/set_mode/offboard', 'std_srvs/srv/Trigger', 'ros2 service call /DRONE_NS/mavros/set_mode/offboard std_srvs/srv/Trigger {}', 'Enables virtual stick/offboard-style control before velocity setpoints.'],
    ['Abort Mission', 'service', '/wildbridge/abort_mission', 'std_srvs/srv/Trigger', 'ros2 service call /DRONE_NS/wildbridge/abort_mission std_srvs/srv/Trigger {}', 'WildBridge-specific stop path for the current mission.'],
  ]],
  ['ROS Setpoint Topics', 'ROS topics you publish to when controlling the drone from ROS. Use /DRONE_NS when the bridge is launched with a namespace such as /mini1.', [
    ['Local Position Setpoint', 'topic', '/mavros/setpoint_position/local', 'geometry_msgs/msg/PoseStamped', 'ros2 topic pub /DRONE_NS/mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "{pose: {position: {x: 5.0, y: 0.0, z: 20.0}}}"', 'Local x/y/z setpoint converted by the bridge into a GPS waypoint using the current home position.'],
    ['Global Position Setpoint', 'topic', '/mavros/setpoint_position/global', 'sensor_msgs/msg/NavSatFix', 'ros2 topic pub /DRONE_NS/mavros/setpoint_position/global sensor_msgs/msg/NavSatFix "{latitude: 55.6761, longitude: 12.5683, altitude: 35.0}"', 'Direct GPS waypoint in decimal degrees and meters.'],
    ['Velocity Setpoint', 'topic', '/mavros/setpoint_velocity/cmd_vel', 'geometry_msgs/msg/TwistStamped', 'ros2 topic pub /DRONE_NS/mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {z: 0.1}}}"', 'Velocity command mapped to virtual stick. Enable offboard/virtual stick first.'],
    ['Attitude/Gimbal Setpoint', 'topic', '/mavros/setpoint_attitude/attitude', 'geometry_msgs/msg/PoseStamped', 'ros2 topic pub /DRONE_NS/mavros/setpoint_attitude/attitude geometry_msgs/msg/PoseStamped "{pose: {orientation: {w: 1.0}}}"', 'Used by the bridge as a gimbal/attitude command path; pitch is extracted for gimbal control.'],
  ]],
  ['ROS Telemetry Topics', 'Topics published by the bridge from phone telemetry. These are not commands, but they are what downstream ROS nodes should subscribe to for state.', [
    ['Connection State', 'topic', '/mavros/state/connected', 'std_msgs/msg/Bool', 'ros2 topic echo /DRONE_NS/mavros/state/connected', 'True when the bridge believes the phone/drone interface is connected.'],
    ['Mode', 'topic', '/mavros/state/mode', 'std_msgs/msg/String', 'ros2 topic echo /DRONE_NS/mavros/state/mode', 'PX4/MAVROS-style mode translated from DJI flight mode.'],
    ['Global Position', 'topic', '/mavros/global_position/global', 'sensor_msgs/msg/NavSatFix', 'ros2 topic echo /DRONE_NS/mavros/global_position/global', 'GPS latitude, longitude, and altitude from phone telemetry.'],
    ['Local Pose', 'topic', '/mavros/local_position/pose', 'geometry_msgs/msg/PoseStamped', 'ros2 topic echo /DRONE_NS/mavros/local_position/pose', 'Flat-earth local pose relative to the home location.'],
    ['Battery', 'topic', '/mavros/battery', 'sensor_msgs/msg/BatteryState', 'ros2 topic echo /DRONE_NS/mavros/battery', 'Battery percentage normalized to 0.0-1.0.'],
    ['WildBridge Distance Home', 'topic', '/wildbridge/distance_to_home', 'std_msgs/msg/Float64', 'ros2 topic echo /DRONE_NS/wildbridge/distance_to_home', 'WildBridge-specific distance-to-home telemetry in meters.'],
  ]],
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
    this.receiverBufferTargetMs = receiverBufferPolicy.initialMs;
    this.receiverBufferSupported = false;
    this.receiverBufferAppliedMs = null;
    this.receiverStableSince = Date.now();
    this.lastPacketsLost = 0;
    this.lastFramesDropped = 0;
    this.lastFreezeCount = 0;
    this.lastJitterBufferDelay = 0;
    this.lastJitterBufferEmittedCount = 0;
    this.lastBytesReceived = 0;
    this.latestSdpRecovery = emptySdpRecovery();

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
      const videoTransceiver = this.pc.addTransceiver('video', { direction: 'recvonly' });
      this.applyReceiverBufferTarget(videoTransceiver.receiver, 'transceiver_created');
      this.pc.ontrack = (event) => {
        this.applyReceiverBufferTarget(event.receiver, 'track_attached');
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
      this.latestSdpRecovery = summarizeSdpRecovery(this.pc.localDescription?.sdp || '', '');
      const response = await fetch(whepUrl, {
        method: 'POST',
        headers: { 'content-type': 'application/sdp' },
        body: this.pc.localDescription.sdp,
      });
      if (!response.ok) throw new Error(`WHEP ${response.status}: ${await response.text()}`);
      this.resourceUrl = response.headers.get('location');
      const answerSdp = await response.text();
      await this.pc.setRemoteDescription({ type: 'answer', sdp: answerSdp });
      this.latestSdpRecovery = summarizeSdpRecovery(this.pc.localDescription?.sdp || '', answerSdp);
      this.report('whep_answer_set', { url: whepUrl, recovery: this.latestSdpRecovery.summary });
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
    this.receiverStableSince = Date.now();
    this.lastPacketsLost = 0;
    this.lastFramesDropped = 0;
    this.lastFreezeCount = 0;
    this.lastJitterBufferDelay = 0;
    this.lastJitterBufferEmittedCount = 0;
    this.lastBytesReceived = 0;
    this.applyReceiverBufferTarget(null, 'stats_started');
    this.statsTimer = setInterval(() => this.collectStats(), 1000);
  }

  applyReceiverBufferTarget(receiver = null, reason = 'update') {
    const targetSeconds = this.receiverBufferTargetMs / 1000;
    const receivers = receiver ? [receiver] : (this.pc?.getReceivers?.() || []);
    let supported = false;
    for (const activeReceiver of receivers) {
      if (activeReceiver?.track?.kind !== 'video') continue;
      if (!('jitterBufferTarget' in activeReceiver)) continue;
      try {
        activeReceiver.jitterBufferTarget = targetSeconds;
        supported = true;
      } catch (error) {
        this.report('receiver_jitter_target_error', { error: error.message, targetMs: this.receiverBufferTargetMs, reason });
      }
    }
    if (supported) {
      const changed = this.receiverBufferAppliedMs !== this.receiverBufferTargetMs;
      this.receiverBufferSupported = true;
      this.receiverBufferAppliedMs = this.receiverBufferTargetMs;
      if (changed) this.report('receiver_jitter_target_set', { targetMs: this.receiverBufferTargetMs, reason });
    }
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
    const codec = inbound.codecId ? stats.get(inbound.codecId) : null;

    const seconds = Math.max((now - this.lastStatsAt) / 1000, 0.001);
    const framesDecoded = inbound.framesDecoded || 0;
    const fps = (framesDecoded - this.lastFramesDecoded) / seconds;
    this.lastFramesDecoded = framesDecoded;
    this.lastStatsAt = now;

    const currentTime = this.video.currentTime || 0;
    const freezeCount = inbound.freezeCount || 0;
    const packetsLost = inbound.packetsLost || 0;
    const framesDropped = inbound.framesDropped || 0;
    const bytesReceived = inbound.bytesReceived || 0;
    const bitrateKbps = this.lastBytesReceived > 0 ? ((bytesReceived - this.lastBytesReceived) * 8) / seconds / 1000 : 0;
    this.lastBytesReceived = bytesReceived;
    const jitterBufferMetrics = this.computeJitterBufferMetrics(inbound);
    this.adaptReceiverBuffer({
      jitterSeconds: inbound.jitter || 0,
      packetsLost,
      framesDropped,
      freezeCount,
      jitterDelayMs: jitterBufferMetrics.delayMs,
    });

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
      framesDropped,
      packetsLost,
      jitter: Number((inbound.jitter || 0).toFixed(4)),
      jitterBufferTargetMs: this.receiverBufferAppliedMs ?? this.receiverBufferTargetMs,
      jitterBufferSupported: this.receiverBufferSupported,
      jitterBufferDelayMs: jitterBufferMetrics.delayMs,
      jitterBufferEmittedCount: jitterBufferMetrics.emittedCount,
      jitterBufferMinimumDelayMs: msOrNull(inbound.jitterBufferMinimumDelay),
      jitterBufferTargetDelayMs: msOrNull(inbound.jitterBufferTargetDelay),
      receiverBufferPolicy: receiverBufferPolicy.mode,
      freezeCount,
      keyFramesDecoded: inbound.keyFramesDecoded || 0,
      pliCount: inbound.pliCount || 0,
      firCount: inbound.firCount || 0,
      nackCount: inbound.nackCount || 0,
      pauseCount: inbound.pauseCount || 0,
      totalFreezesDuration: Number((inbound.totalFreezesDuration || 0).toFixed(3)),
      bytesReceived,
      bitrateKbps: Number(Math.max(0, bitrateKbps).toFixed(1)),
      frameWidth: inbound.frameWidth || this.video.videoWidth || 0,
      frameHeight: inbound.frameHeight || this.video.videoHeight || 0,
      codecMimeType: codec?.mimeType || '',
      codecPayloadType: codec?.payloadType ?? null,
      codecFmtpLine: codec?.sdpFmtpLine || '',
      sdpRecovery: this.latestSdpRecovery,
      connectionLosses: this.connectionLosses,
      stalls: this.stalls,
      decodeErrors: this.decodeErrors,
    };
    this.updateStats(payload);
    if (selectedDroneName === this.drone.name) updateModalForDrone(this.drone.name);
    addChartSample(this.drone.name, {
      fps: payload.fps,
      bitrateKbps: payload.bitrateKbps,
      framesDropped: payload.framesDropped,
      packetsLost: payload.packetsLost,
      jitter: payload.jitter,
      jitterBufferTargetMs: payload.jitterBufferTargetMs,
      jitterBufferDelayMs: payload.jitterBufferDelayMs,
      freezeCount: payload.freezeCount,
      nackCount: payload.nackCount,
      pliCount: payload.pliCount,
      keyFramesDecoded: payload.keyFramesDecoded,
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
    setText(this.tile, '[data-stat="jitterBufferTarget"]', stats.jitterBufferSupported ? `${stats.jitterBufferTargetMs}ms` : 'n/a');
    setText(this.tile, '[data-stat="jitter"]', stats.jitter);
    setText(this.tile, '[data-stat="size"]', stats.frameWidth && stats.frameHeight ? `${stats.frameWidth}x${stats.frameHeight}` : '-');
    this.options.onStats?.(stats);
  }

  computeJitterBufferMetrics(inbound) {
    const emittedCount = inbound.jitterBufferEmittedCount || 0;
    const delay = inbound.jitterBufferDelay || 0;
    const emittedDelta = Math.max(0, emittedCount - this.lastJitterBufferEmittedCount);
    const delayDelta = Math.max(0, delay - this.lastJitterBufferDelay);
    this.lastJitterBufferEmittedCount = emittedCount;
    this.lastJitterBufferDelay = delay;
    const delayMs = emittedDelta > 0 ? Number(((delayDelta / emittedDelta) * 1000).toFixed(1)) : null;
    return { delayMs, emittedCount };
  }

  adaptReceiverBuffer(metrics) {
    const now = Date.now();
    const lostDelta = Math.max(0, metrics.packetsLost - this.lastPacketsLost);
    const dropDelta = Math.max(0, metrics.framesDropped - this.lastFramesDropped);
    const freezeDelta = Math.max(0, metrics.freezeCount - this.lastFreezeCount);
    this.lastPacketsLost = metrics.packetsLost;
    this.lastFramesDropped = metrics.framesDropped;
    this.lastFreezeCount = metrics.freezeCount;

    const jitterMs = metrics.jitterSeconds * 1000;
    const latePressure = lostDelta > 0 || dropDelta > 0 || freezeDelta > 0 || jitterMs > this.receiverBufferTargetMs * 0.8;
    if (latePressure) {
      this.receiverStableSince = now;
      const nextTarget = Math.min(receiverBufferPolicy.maxMs, this.receiverBufferTargetMs + receiverBufferPolicy.stepMs);
      if (nextTarget !== this.receiverBufferTargetMs) {
        this.receiverBufferTargetMs = nextTarget;
        this.applyReceiverBufferTarget(null, 'late_pressure');
      }
      return;
    }

    if (now - this.receiverStableSince < receiverBufferPolicy.stableAfterMs) return;
    const nextTarget = Math.max(receiverBufferPolicy.minMs, this.receiverBufferTargetMs - receiverBufferPolicy.stepMs);
    if (nextTarget !== this.receiverBufferTargetMs) {
      this.receiverBufferTargetMs = nextTarget;
      this.receiverStableSince = now;
      this.applyReceiverBufferTarget(null, 'stable_decay');
    }
  }

  report(type, extra = {}) {
    const message = `${new Date().toLocaleTimeString()} ${type}`;
    if (this.events) this.events.textContent = message;
    if (selectedDroneName === this.drone.name) modalEvent.textContent = `${message}\n${JSON.stringify(extra, null, 2)}`;
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
    if (!this.drone.mediaMtx?.ready || this.reconnectTimer) return;
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

function emptySdpRecovery() {
  return summarizeSdpRecovery('', '');
}

function summarizeSdpRecovery(localSdp, remoteSdp) {
  const local = parseSdpRecovery(localSdp);
  const remote = parseSdpRecovery(remoteSdp);
  const negotiated = remoteSdp ? remote : local;
  const missing = [];
  if (!negotiated.nack) missing.push('NACK');
  if (!negotiated.pli) missing.push('PLI');
  if (!negotiated.fir) missing.push('FIR');
  if (!negotiated.rtx) missing.push('RTX');
  const health = !remoteSdp ? 'pending' : (negotiated.nack && (negotiated.pli || negotiated.fir) ? (negotiated.rtx ? 'strong' : 'usable') : 'weak');
  return {
    health,
    summary: remoteSdp ? `${health}; ${missing.length ? `missing ${missing.join(', ')}` : 'NACK/PLI/FIR/RTX present'}` : 'waiting for WHEP answer',
    negotiated,
    local,
    remote,
  };
}

function parseSdpRecovery(sdp) {
  const result = {
    nack: false,
    pli: false,
    fir: false,
    rtx: false,
    h264: false,
    h264PayloadTypes: [],
    rtxPayloadTypes: [],
    codecs: [],
    h264Profiles: [],
    h264PacketizationModes: [],
  };
  if (!sdp) return result;
  const fmtpByPayload = new Map();
  for (const rawLine of sdp.split(/\r?\n/)) {
    const line = rawLine.trim();
    const rtcpFb = line.match(/^a=rtcp-fb:(\*|\d+)\s+(.+)$/i);
    if (rtcpFb) {
      const feedback = rtcpFb[2].toLowerCase();
      if (feedback.includes('nack')) result.nack = true;
      if (feedback.includes('nack pli')) result.pli = true;
      if (feedback.includes('ccm fir')) result.fir = true;
    }
    const rtpMap = line.match(/^a=rtpmap:(\d+)\s+([^/\s]+)(?:\/([^\s]+))?/i);
    if (rtpMap) {
      const payloadType = rtpMap[1];
      const codec = rtpMap[2].toUpperCase();
      result.codecs.push(`${payloadType}:${codec}`);
      if (codec === 'RTX') {
        result.rtx = true;
        result.rtxPayloadTypes.push(payloadType);
      }
      if (codec === 'H264') {
        result.h264 = true;
        result.h264PayloadTypes.push(payloadType);
      }
    }
    const fmtp = line.match(/^a=fmtp:(\d+)\s+(.+)$/i);
    if (fmtp) fmtpByPayload.set(fmtp[1], fmtp[2]);
  }
  for (const payloadType of result.h264PayloadTypes) {
    const fmtp = fmtpByPayload.get(payloadType) || '';
    const profile = fmtp.match(/profile-level-id=([^;\s]+)/i)?.[1];
    const packetization = fmtp.match(/packetization-mode=([^;\s]+)/i)?.[1];
    if (profile && !result.h264Profiles.includes(profile)) result.h264Profiles.push(profile);
    if (packetization && !result.h264PacketizationModes.includes(packetization)) result.h264PacketizationModes.push(packetization);
  }
  return result;
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
  updateSummary({ detected: state.drones.length, active: activeDrones, discovered, telemetry: connected, streams: readyStreams, browser: browserConnections, logFile: state.logFile });
  emptyState.hidden = state.drones.length > 0;
  renderVideoTiles(state);
  renderHealthPanels(state);
  renderTelemetryPanels(state);
  renderPublishTargets(state);
  renderPublishCatalog();
  if (selectedDroneName) updateModalForDrone(selectedDroneName);
  updateTelemetryChartSamples(state);
  updateCharts();
}

function renderVideoTiles(state) {
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
}

function renderHealthPanels(state) {
  for (const drone of state.drones) {
    let card = healthCards.get(drone.name);
    if (!card) {
      card = createHealthCard(drone.name);
      healthCards.set(drone.name, card);
      healthGrid.appendChild(card.element);
    }
    updateHealthCard(card, drone);
  }
  for (const [name, card] of healthCards.entries()) {
    if (!state.drones.some((drone) => drone.name === name)) {
      card.element.remove();
      healthCards.delete(name);
      healthTrendState.delete(name);
    }
  }
}

function createHealthCard(name) {
  const element = document.createElement('article');
  element.className = 'healthCard';
  element.tabIndex = 0;
  element.setAttribute('role', 'button');
  element.setAttribute('aria-label', `Open details for ${name}`);
  element.innerHTML = `<div class="healthHeader"><div><h3>${escapeHtml(name)}</h3><p data-role="subtitle">Health waiting</p></div><div class="telemetryPills" data-role="pills"></div></div><div class="healthSummary" data-role="health"></div><dl class="healthMetrics" data-role="metrics"></dl>`;
  element.addEventListener('click', () => openDroneModal(name));
  element.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      openDroneModal(name);
    }
  });
  return { element, subtitle: element.querySelector('[data-role="subtitle"]'), pills: element.querySelector('[data-role="pills"]'), health: element.querySelector('[data-role="health"]'), metrics: element.querySelector('[data-role="metrics"]') };
}

function updateHealthCard(card, drone) {
  const telemetry = drone.lastTelemetry || {};
  const phone = telemetry.phoneLocation || {};
  const sender = telemetry.webRtc || {};
  const media = drone.mediaMtx || {};
  const browser = drone.browserStats || {};
  const recovery = browser.sdpRecovery || emptySdpRecovery();
  const trend = updateHealthTrend(drone);
  const health = buildDroneHealth(drone, trend);
  card.element.classList.toggle('ignored', !!drone.ignored);
  updateText(card.subtitle, `${drone.ip || 'no ip'} | ${health.primary}`);
  card.pills.replaceChildren(
    makePill(health.label, health.tone),
    makePill(drone.telemetryConnected ? 'telemetry ok' : 'telemetry down', drone.telemetryConnected ? 'good' : 'bad'),
    makePill(media.ready ? 'stream ready' : 'stream wait', media.ready ? 'good' : 'warn'),
    makePill(browser.connectionState || 'browser idle', browser.connectionState === 'connected' ? 'good' : 'warn'),
    makePill(drone.ignored ? 'ignored' : 'active', drone.ignored ? 'warn' : 'good'),
  );
  renderHealthSummary(card.health, health);
  updateDetailList(card.metrics, [
    ['wifi', 'Wi-Fi RSSI', phone.wifiRssi === undefined ? '-' : `${phone.wifiRssi} dBm`],
    ['phoneBattery', 'Phone Battery', phone.battery === undefined ? '-' : `${phone.battery}%`],
    ['senderFps', 'Sender FPS', sender.outputFps === undefined ? '-' : `${Number(sender.outputFps).toFixed(1)} / ${sender.targetFps || '-'}`],
    ['senderProcessing', 'Sender Processing', sender.averageFrameProcessingMs === undefined ? '-' : `${Number(sender.averageFrameProcessingMs).toFixed(1)} ms/frame`],
    ['browserFps', 'Browser FPS', browser.fps === undefined ? '-' : Number(browser.fps).toFixed(1)],
    ['bitrate', 'Bitrate', browser.bitrateKbps === undefined ? '-' : `${browser.bitrateKbps} kbps`],
    ['loss', 'Loss / Drop', `${browser.packetsLost ?? '-'} / ${browser.framesDropped ?? '-'}`],
    ['recentLoss', 'Recent Loss / Drop', trend.ready ? `${trend.lostDelta} / ${trend.dropDelta}` : '-'],
    ['jitter', 'Jitter', browser.jitter === undefined ? '-' : `${Number(browser.jitter * 1000).toFixed(0)} ms`],
    ['freezes', 'Freezes', browser.freezeCount ?? '-'],
    ['recentFreezes', 'Recent Freezes', trend.ready ? trend.freezeDelta : '-'],
    ['recovery', 'Recovery', recovery.summary || '-'],
  ]);
}

function updateHealthTrend(drone) {
  const browser = drone.browserStats || {};
  const media = drone.mediaMtx || {};
  const now = Date.now();
  const previous = healthTrendState.get(drone.name);
  const hasBrowserStats = !!drone.browserStats;
  const current = {
    mediaErrors: Number(media.inboundFramesInError || 0),
    packetsLost: hasBrowserStats ? Number(browser.packetsLost || 0) : Number(previous?.packetsLost || 0),
    framesDropped: hasBrowserStats ? Number(browser.framesDropped || 0) : Number(previous?.framesDropped || 0),
    freezeCount: hasBrowserStats ? Number(browser.freezeCount || 0) : Number(previous?.freezeCount || 0),
  };
  if (!previous) {
    const initialTrend = { ready: false, seconds: 0, inboundErrorDelta: 0, lostDelta: 0, dropDelta: 0, freezeDelta: 0 };
    healthTrendState.set(drone.name, { ...current, at: now, lastTrend: initialTrend });
    return initialTrend;
  }
  const delta = (key) => Math.max(0, current[key] - Number(previous[key] || 0));
  const trend = {
    ready: true,
    seconds: Math.max((now - previous.at) / 1000, 0.001),
    inboundErrorDelta: delta('mediaErrors'),
    lostDelta: delta('packetsLost'),
    dropDelta: delta('framesDropped'),
    freezeDelta: delta('freezeCount'),
  };
  healthTrendState.set(drone.name, { ...current, at: now, lastTrend: trend });
  return trend;
}

function renderTelemetryPanels(state) {
  for (const drone of state.drones) {
    let card = telemetryCards.get(drone.name);
    if (!card) {
      card = createTelemetryCard(drone.name);
      telemetryCards.set(drone.name, card);
      telemetryGrid.appendChild(card.element);
    }
    updateTelemetryCard(card, drone);
  }
  for (const [name, card] of telemetryCards.entries()) {
    if (!state.drones.some((drone) => drone.name === name)) {
      card.element.remove();
      telemetryCards.delete(name);
    }
  }
}

function createTelemetryCard(name) {
  const element = document.createElement('article');
  element.className = 'telemetryCard';
  element.innerHTML = `<div class="telemetryHeader"><div><h3>${escapeHtml(name)}</h3><p data-role="subtitle">Telemetry waiting</p></div><div class="telemetryPills" data-role="pills"></div></div><div class="telemetryTree" data-role="tree"></div>`;
  return { element, subtitle: element.querySelector('[data-role="subtitle"]'), pills: element.querySelector('[data-role="pills"]'), tree: element.querySelector('[data-role="tree"]'), treeState: new Map() };
}

function updateTelemetryCard(card, drone) {
  const media = drone.mediaMtx || {};
  updateText(card.subtitle, `${drone.ip || 'no ip'} | telemetry ${drone.telemetryConnected ? 'ok' : 'down'} | ${telemetryAgeLabel(drone)}`);
  card.pills.replaceChildren(
    makePill(drone.telemetryConnected ? 'telemetry ok' : 'telemetry down', drone.telemetryConnected ? 'good' : 'bad'),
    makePill(media.ready ? 'stream ready' : 'stream wait', media.ready ? 'good' : 'warn'),
    makePill(drone.browserStats?.connectionState || 'browser idle', drone.browserStats?.connectionState === 'connected' ? 'good' : 'warn'),
    makePill(drone.ignored ? 'ignored' : 'active', drone.ignored ? 'warn' : 'good'),
  );
  rememberTreeState(card);
  card.tree.replaceChildren(renderTreeNode('root', {
    meta: {
      ip: drone.ip,
      discoveredIp: drone.discoveredIp,
      status: drone.status,
      telemetryConnected: drone.telemetryConnected,
      telemetryPackets: drone.telemetryPackets,
      telemetryReconnects: drone.telemetryReconnects,
      lastTelemetryAt: drone.lastTelemetryAt,
      lastDiscoveryAt: drone.lastDiscoveryAt,
      lastError: drone.lastError,
      ignored: drone.ignored,
    },
    telemetry: drone.lastTelemetry || {},
    senderWebRtc: drone.lastTelemetry?.webRtc || {},
    mediaMtx: {
      ready: media.ready,
      readers: media.readers,
      tracks: media.tracks,
      bytesReceived: media.bytesReceived,
      bytesSent: media.bytesSent,
      inboundFramesInError: media.inboundFramesInError,
    },
    browser: drone.browserStats || {},
  }, { open: true, hideLabel: true, path: 'root', state: card.treeState }));
}

function rememberTreeState(card) {
  card.tree.querySelectorAll('details.treeNode[data-tree-path]').forEach((details) => {
    card.treeState.set(details.dataset.treePath, details.open);
  });
}

function renderPublishTargets(state) {
  publishTargets.replaceChildren();
  for (const drone of state.drones) {
    const card = document.createElement('article');
    card.className = 'targetCard';
    const baseUrl = drone.ip ? `http://${drone.ip}:8080` : 'waiting for IP';
    card.innerHTML = `<div class="targetCardHeader"><div><h3>${escapeHtml(drone.name)}</h3><p>${escapeHtml(drone.status || 'unknown')} | telemetry ${drone.telemetryConnected ? 'ok' : 'down'}</p></div><div class="targetPills"></div></div><div class="targetRow"><span class="treeMeta">Command base</span><span class="targetUrl">${escapeHtml(baseUrl)}</span></div><div class="targetRow"><span class="treeMeta">Config</span><span class="targetUrl">${escapeHtml(drone.ip ? `${baseUrl}/config` : 'waiting for IP')}</span></div>`;
    card.querySelector('.targetPills').append(
      makePill(drone.ip ? 'reachable IP' : 'no IP', drone.ip ? 'good' : 'warn'),
      makePill(drone.ignored ? 'ignored' : 'armed for test', drone.ignored ? 'warn' : 'good'),
    );
    publishTargets.appendChild(card);
  }
}

function renderPublishCatalog() {
  if (publishGrid.childElementCount) return;
  renderHttpPublishCatalog();
  renderRosPublishCatalog();
}

function renderHttpPublishCatalog() {
  for (const [groupTitle, description, commands] of publishCatalog) {
    const card = document.createElement('section');
    card.className = 'publishCard';
    card.innerHTML = `<div class="publishCardHeader"><div><h3>${escapeHtml(groupTitle)}</h3><p>${escapeHtml(description)}</p></div><span class="publishChannel">HTTP</span></div><div class="publishCardBody"></div>`;
    const body = card.querySelector('.publishCardBody');
    for (const [title, method, endpoint, commandBody, commandDescription] of commands) {
      const item = document.createElement('article');
      item.className = 'publishItem';
      const requestLine = method === 'GET' ? `${method} http://DRONE_IP:8080${endpoint}` : `${method} http://DRONE_IP:8080${endpoint}\nBody: ${commandBody || '(empty)'}`;
      item.innerHTML = `<div class="publishItemHeader"><h4>${escapeHtml(title)}</h4><span class="publishMeta">${escapeHtml(method)} ${escapeHtml(endpoint)}</span></div><p class="publishDescription">${escapeHtml(commandDescription)}</p><pre class="publishCode">${escapeHtml(requestLine)}</pre>`;
      body.appendChild(item);
    }
    publishGrid.appendChild(card);
  }
}

function renderRosPublishCatalog() {
  for (const [groupTitle, description, commands] of rosPublishCatalog) {
    const card = document.createElement('section');
    card.className = 'publishCard rosPublishCard';
    card.innerHTML = `<div class="publishCardHeader"><div><h3>${escapeHtml(groupTitle)}</h3><p>${escapeHtml(description)}</p></div><span class="publishChannel ros">ROS 2</span></div><div class="publishCardBody"></div>`;
    const body = card.querySelector('.publishCardBody');
    for (const [title, kind, endpoint, typeName, example, commandDescription] of commands) {
      const item = document.createElement('article');
      item.className = 'publishItem';
      item.innerHTML = `<div class="publishItemHeader"><h4>${escapeHtml(title)}</h4><span class="publishMeta">${escapeHtml(kind)} ${escapeHtml(endpoint)}</span></div><p class="publishDescription">${escapeHtml(commandDescription)}</p><div class="publishType">${escapeHtml(typeName)}</div><pre class="publishCode">${escapeHtml(example)}</pre>`;
      body.appendChild(item);
    }
    publishGrid.appendChild(card);
  }
}

function renderHealthSummary(root, health) {
  if (!root) return;
  const issues = health.issues.length ? health.issues : [{ label: 'No active symptom detected', tone: 'good', detail: health.primary }];
  const visibleIssues = issues.slice(0, 3).map((issue) => {
    const item = document.createElement('div');
    item.className = `healthIssue ${issue.tone || 'warn'}`;
    item.innerHTML = `<strong>${escapeHtml(issue.label)}</strong><span>${escapeHtml(issue.detail || '-')}</span>`;
    return item;
  });
  if (issues.length > visibleIssues.length) {
    const more = document.createElement('div');
    more.className = 'healthIssue more';
    more.innerHTML = `<strong>${issues.length - visibleIssues.length} more indicators</strong><span>Open details for the full telemetry and recovery view.</span>`;
    visibleIssues.push(more);
  }
  root.replaceChildren(...visibleIssues);
}

function buildDroneHealth(drone, trend = { ready: false, inboundErrorDelta: 0, lostDelta: 0, dropDelta: 0, freezeDelta: 0 }) {
  const telemetry = drone.lastTelemetry || {};
  const phone = telemetry.phoneLocation || {};
  const sender = telemetry.webRtc || {};
  const media = drone.mediaMtx || {};
  const browser = drone.browserStats || {};
  const recovery = browser.sdpRecovery?.negotiated || {};
  const issues = [];

  const addIssue = (label, tone, detail) => issues.push({ label, tone, detail });
  const wifiRssi = Number(phone.wifiRssi);
  const phoneBattery = Number(phone.battery);
  const senderTargetFps = Number(sender.targetFps || sender.configuredFps || 0);
  const senderOutputFps = Number(sender.outputFps || 0);
  const senderInputFps = Number(sender.inputFps || 0);
  const senderProcessingMs = Number(sender.averageFrameProcessingMs || 0);
  const browserFps = Number(browser.fps || 0);
  const browserJitterMs = Number(browser.jitter || 0) * 1000;
  const browserBitrateKbps = Number(browser.bitrateKbps || 0);
  const framesDecoded = Number(browser.framesDecoded || 0);
  const packetsLost = Number(browser.packetsLost || 0);
  const framesDropped = Number(browser.framesDropped || 0);
  const freezes = Number(browser.freezeCount || 0);
  const inboundErrors = Number(media.inboundFramesInError || 0);
  const recentLoss = Number(trend.lostDelta || 0);
  const recentDrops = Number(trend.dropDelta || 0);
  const recentFreezes = Number(trend.freezeDelta || 0);
  const recentInboundErrors = Number(trend.inboundErrorDelta || 0);

  if (!drone.telemetryConnected) addIssue('Telemetry down', 'bad', 'Phone telemetry socket is not producing samples.');
  if (drone.telemetryConnected && !media.ready && (drone.telemetryPackets || 0) > 20) addIssue('Stream not ready', 'bad', 'Telemetry is alive but MediaMTX has no ready video path.');
  if (Number.isFinite(wifiRssi)) {
    if (wifiRssi <= -78) addIssue('Wi-Fi weak', 'bad', `${wifiRssi} dBm can produce loss and jitter.`);
    else if (wifiRssi <= -67) addIssue('Wi-Fi marginal', 'warn', `${wifiRssi} dBm leaves limited airtime margin.`);
  }
  if (Number.isFinite(phoneBattery) && phoneBattery >= 0) {
    if (phoneBattery <= 15) addIssue('Phone battery low', 'bad', `${phoneBattery}% may trigger thermal or power limits.`);
    else if (phoneBattery <= 30) addIssue('Phone battery watch', 'warn', `${phoneBattery}% remaining.`);
  }
  if (senderTargetFps > 0) {
    const frameBudgetMs = 1000 / senderTargetFps;
    if (senderOutputFps > 0 && senderOutputFps < senderTargetFps * 0.55) addIssue('Sender FPS low', 'bad', `Phone output ${senderOutputFps.toFixed(1)} / ${senderTargetFps} fps.`);
    else if (senderInputFps > 0 && senderOutputFps < senderInputFps * 0.65) addIssue('Sender dropping frames', 'warn', `Input ${senderInputFps.toFixed(1)} fps, output ${senderOutputFps.toFixed(1)} fps.`);
    if (senderProcessingMs >= frameBudgetMs * 0.9) addIssue('Encoder pipeline saturated', 'bad', `${senderProcessingMs.toFixed(1)} ms/frame near ${frameBudgetMs.toFixed(1)} ms budget.`);
    else if (senderProcessingMs >= frameBudgetMs * 0.7) addIssue('Encoder pipeline hot', 'warn', `${senderProcessingMs.toFixed(1)} ms/frame resize/processing.`);
  }
  if (browser.connectionState === 'connected') {
    if (senderOutputFps > 2 && browserFps > 0 && browserFps < senderOutputFps * 0.55) addIssue('Browser decode lag', 'bad', `Browser ${browserFps.toFixed(1)} fps while sender outputs ${senderOutputFps.toFixed(1)} fps.`);
    else if (media.ready && framesDecoded > 0 && browserFps <= 1) addIssue('Browser video stalled', 'bad', `Browser is connected but decoding only ${browserFps.toFixed(1)} fps.`);
    else if (media.ready && browserBitrateKbps > 0 && browserBitrateKbps < 80) addIssue('Browser bitrate low', 'warn', `Receiving ${browserBitrateKbps.toFixed(0)} kbps from a ready H264 stream.`);
    if (browserJitterMs >= 150) addIssue('Browser jitter high', 'bad', `${browserJitterMs.toFixed(0)} ms RTP jitter.`);
    else if (browserJitterMs >= 60) addIssue('Browser jitter elevated', 'warn', `${browserJitterMs.toFixed(0)} ms RTP jitter.`);
    if (recentLoss > 0) addIssue('Packet loss active', recentLoss > 20 ? 'bad' : 'warn', `${recentLoss} new lost packets; ${packetsLost} cumulative.`);
    if (recentDrops > 0) addIssue('Frame drops active', recentDrops > 10 ? 'bad' : 'warn', `${recentDrops} new dropped frames; ${framesDropped} cumulative.`);
    if (recentFreezes > 0) addIssue('Receiver freezes active', recentFreezes > 2 ? 'bad' : 'warn', `${recentFreezes} new freeze events; ${freezes} cumulative.`);
  }
  if (recentInboundErrors > 0) addIssue('MediaMTX frame errors active', recentInboundErrors > 5 ? 'bad' : 'warn', `${recentInboundErrors} new inbound frame errors; ${inboundErrors} cumulative.`);
  if (browser.sdpRecovery) {
    if (!recovery.nack) addIssue('NACK missing', 'warn', 'Negotiated SDP does not advertise generic NACK.');
    if (!recovery.pli && !recovery.fir) addIssue('Keyframe request weak', 'warn', 'Negotiated SDP lacks PLI/FIR feedback.');
    if (!recovery.rtx) addIssue('RTX missing', 'warn', 'Retransmission payloads were not negotiated.');
  }

  const worstTone = issues.some((issue) => issue.tone === 'bad') ? 'bad' : (issues.some((issue) => issue.tone === 'warn') ? 'warn' : 'good');
  const label = worstTone === 'good' ? 'health ok' : (worstTone === 'bad' ? 'health bad' : 'health watch');
  const primary = issues[0]?.detail || 'Telemetry, stream, sender, browser, and recovery indicators look nominal.';
  return { label, tone: worstTone, primary, issues };
}

function cameraLostMessage(drone) {
  if (drone.mediaMtx?.ready || !drone.telemetryConnected || (drone.telemetryPackets || 0) < 20) return '';
  return 'Device may have been idle too long and the camera feed was lost. Power-cycle the drone and let it cool down before retrying.';
}

function setText(root, selector, value) { updateText(root?.querySelector(selector), value); }
function updateText(element, value) {
  if (!element) return;
  const next = String(value ?? '-');
  if (element.textContent !== next) element.textContent = next;
}
function updateClass(element, className) { if (element && element.className !== className) element.className = className; }

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
    if (rawValue === null || rawValue === undefined) continue;
    const value = Number(rawValue);
    if (!Number.isFinite(value)) continue;
    const series = droneHistory[key] || [];
    const last = series[series.length - 1];
    if (last?.time === time) last.value = value;
    else series.push({ time, value });
    if (series.length > 600) series.splice(0, series.length - 600);
    droneHistory[key] = series;
  }
}

function updateTelemetryChartSamples(state) {
  for (const drone of state.drones) {
    if (drone.ignored) continue;
    const currentPackets = Number(drone.telemetryPackets || 0);
    const telemetry = drone.lastTelemetry || {};
    const speed = telemetry.speed || {};
    const phone = telemetry.phoneLocation || {};
    const previous = telemetryRateState.get(drone.name) || { packets: currentPackets, at: Date.now() };
    const now = Date.now();
    const seconds = Math.max((now - previous.at) / 1000, 0.001);
    telemetryRateState.set(drone.name, { packets: currentPackets, at: now });
    addChartSample(drone.name, {
      telemetryRate: Math.max(0, (currentPackets - previous.packets) / seconds),
      inboundFramesInError: drone.mediaMtx?.inboundFramesInError || 0,
      batteryLevel: telemetry.batteryLevel,
      satelliteCount: telemetry.satelliteCount,
      altitude: telemetry.location?.altitude,
      distanceToHome: telemetry.distanceToHome,
      speedMagnitude: magnitude(speed.x, speed.y, speed.z),
      phoneBattery: phone.battery,
      wifiRssi: phone.wifiRssi,
    });
  }
}

function createSeries(chart, options) {
  if (chart.addSeries && window.LightweightCharts?.LineSeries) return chart.addSeries(window.LightweightCharts.LineSeries, options);
  return chart.addLineSeries(options);
}

function ensureCharts(groupName) {
  const group = chartGroups[groupName];
  const chartsGrid = group?.grid;
  if (!group || !chartsGrid) return false;
  if (!window.LightweightCharts) {
    chartsGrid.innerHTML = '<div class="emptyState">Charts are unavailable because Lightweight Charts did not load. Check network access to the CDN.</div>';
    return false;
  }
  for (const definition of group.definitions) {
    const chartKey = `${groupName}:${definition.key}`;
    if (chartInstances.has(chartKey)) continue;
    const card = document.createElement('section');
    card.className = 'chartCard';
    card.innerHTML = `<div class="chartTitle"><h3>${definition.title}</h3><span>${definition.unit}</span></div><div class="chartCanvas"></div>`;
    chartsGrid.appendChild(card);
    const chart = window.LightweightCharts.createChart(card.querySelector('.chartCanvas'), {
      autoSize: true,
      layout: { background: { color: '#1a1d20' }, textColor: '#edf1f4' },
      grid: { vertLines: { color: '#2b3137' }, horzLines: { color: '#2b3137' } },
      timeScale: { timeVisible: true, secondsVisible: true, borderColor: '#343b42' },
      rightPriceScale: { borderColor: '#343b42' },
      crosshair: { mode: 1 },
    });
    chartInstances.set(chartKey, { chart, card, series: new Map() });
  }
  return true;
}

function updateCharts(groupName = null) {
  const groups = groupName ? [groupName] : Object.keys(chartGroups).filter((name) => document.querySelector(`[data-panel="${name}"]`)?.classList.contains('active'));
  if (!groups.length || !latestState) return;
  for (const activeGroup of groups) updateChartGroup(activeGroup);
}

function updateChartGroup(groupName) {
  const group = chartGroups[groupName];
  if (!group || !ensureCharts(groupName)) return;
  const activeDrones = latestState.drones.filter((drone) => !drone.ignored);
  for (const definition of group.definitions) {
    const instance = chartInstances.get(`${groupName}:${definition.key}`);
    if (!instance) continue;
    for (const drone of activeDrones) {
      let series = instance.series.get(drone.name);
      if (!series) {
        series = createSeries(instance.chart, { title: drone.name, color: droneColor(drone.name), lineWidth: 2, lastValueVisible: true, priceLineVisible: false });
        instance.series.set(drone.name, series);
      }
      series.setData(chartHistory.get(drone.name)?.[definition.key] || []);
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

function getDrone(name) { return latestState?.drones.find((drone) => drone.name === name) || null; }

function updateDetailList(root, rows) {
  if (!root) return;
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

function openModalDetails() { droneModal.querySelectorAll('details').forEach((details) => { details.open = true; }); }

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
  const sender = telemetry.webRtc || {};
  const media = drone.mediaMtx || {};
  const browserStats = modalMountedPlayer?.latestStats || drone.browserStats || {};
  const recovery = browserStats.sdpRecovery || emptySdpRecovery();
  const negotiatedRecovery = recovery.negotiated || {};
  const health = buildDroneHealth(drone, healthTrendState.get(drone.name)?.lastTrend);
  const lostMessage = cameraLostMessage(drone);
  const speed = telemetry.speed || {};
  updateText(modalTitle, drone.name);
  updateText(modalSubtitle, `${drone.ip || 'no ip'} | ${drone.ignored ? 'ignored' : 'active'} | telemetry ${drone.telemetryConnected ? 'ok' : 'down'} | stream ${media.ready ? 'ready' : 'waiting'}`);
  updateDetailList(modalTelemetry, [
    ['telemetry', 'Telemetry', drone.telemetryConnected ? `ok (${telemetryAgeLabel(drone)})` : `down (${telemetryAgeLabel(drone)})`, drone.telemetryConnected ? 'status-good' : 'status-bad'],
    ['packets', 'Packets', drone.telemetryPackets],
    ['battery', 'Battery', telemetry.batteryLevel === undefined ? '-' : `${telemetry.batteryLevel}%`],
    ['flightMode', 'Flight Mode', telemetry.flightMode],
    ['location', 'Location', telemetry.location ? `${telemetry.location.latitude}, ${telemetry.location.longitude}, ${telemetry.location.altitude}m` : '-'],
    ['speed', 'Speed', Number.isFinite(magnitude(speed.x, speed.y, speed.z)) ? `${magnitude(speed.x, speed.y, speed.z).toFixed(2)} m/s` : '-'],
    ['phoneBattery', 'Phone Battery', telemetry.phoneLocation?.battery === undefined ? '-' : `${telemetry.phoneLocation.battery}%`],
    ['wifiRssi', 'Wi-Fi RSSI', telemetry.phoneLocation?.wifiRssi],
    ['senderFps', 'Sender FPS', sender.outputFps === undefined ? '-' : `${Number(sender.outputFps).toFixed(1)} / ${sender.targetFps || '-'}`],
    ['senderProcessing', 'Sender Processing', sender.averageFrameProcessingMs === undefined ? '-' : `${Number(sender.averageFrameProcessingMs).toFixed(1)} ms/frame`],
    ['lastError', 'Last Error', drone.lastError || '-'],
  ]);
  updateDetailList(modalStats, [
    ['health', 'Health', health.label, `status-${health.tone}`],
    ['healthPrimary', 'Primary Symptom', health.issues[0]?.label || 'No active symptom'],
    ['mediamtx', 'MediaMTX', media.ready ? `ready (${media.readers?.length || 0} readers)` : 'waiting', media.ready ? 'status-good' : 'status-warn'],
    ['operatorMessage', 'Operator Message', lostMessage || '-'],
    ['codec', 'Codec', media.tracks?.join(', ') || '-'],
    ['inboundErrors', 'Inbound Errors', media.inboundFramesInError ?? '-'],
    ['fps', 'FPS', browserStats.fps === undefined ? '-' : Number(browserStats.fps).toFixed(1)],
    ['bitrate', 'Receive Bitrate', browserStats.bitrateKbps === undefined ? '-' : `${browserStats.bitrateKbps} kbps`],
    ['decodedFrames', 'Decoded Frames', browserStats.framesDecoded],
    ['droppedFrames', 'Dropped Frames', browserStats.framesDropped],
    ['packetsLost', 'Packets Lost', browserStats.packetsLost],
    ['jitter', 'Jitter', browserStats.jitter],
    ['receiverPolicy', 'Receiver Policy', browserStats.receiverBufferPolicy || '-'],
    ['jitterTarget', 'Jitter Target', browserStats.jitterBufferSupported ? `${browserStats.jitterBufferTargetMs} ms` : 'not supported'],
    ['jitterDelay', 'Jitter Delay', browserStats.jitterBufferDelayMs === null || browserStats.jitterBufferDelayMs === undefined ? '-' : `${browserStats.jitterBufferDelayMs} ms/frame`],
    ['freezes', 'Freezes', browserStats.freezeCount],
    ['connectionLosses', 'Connection Losses', browserStats.connectionLosses],
    ['stalls', 'Stalls', browserStats.stalls],
    ['decodeErrors', 'Decode Errors', browserStats.decodeErrors],
  ]);
  updateDetailList(modalRecovery, [
    ['sdpHealth', 'SDP Recovery', recovery.summary || '-'],
    ['nack', 'NACK Negotiated', negotiatedRecovery.nack ? 'yes' : 'no', negotiatedRecovery.nack ? 'status-good' : 'status-warn'],
    ['pli', 'PLI Negotiated', negotiatedRecovery.pli ? 'yes' : 'no', negotiatedRecovery.pli ? 'status-good' : 'status-warn'],
    ['fir', 'FIR Negotiated', negotiatedRecovery.fir ? 'yes' : 'no', negotiatedRecovery.fir ? 'status-good' : 'status-warn'],
    ['rtx', 'RTX Negotiated', negotiatedRecovery.rtx ? 'yes' : 'no', negotiatedRecovery.rtx ? 'status-good' : 'status-warn'],
    ['h264Profile', 'H264 Profile', negotiatedRecovery.h264Profiles?.join(', ') || browserStats.codecFmtpLine || '-'],
    ['packetization', 'Packetization', negotiatedRecovery.h264PacketizationModes?.join(', ') || '-'],
    ['nackCount', 'NACK Count', browserStats.nackCount ?? '-'],
    ['pliCount', 'PLI Count', browserStats.pliCount ?? '-'],
    ['firCount', 'FIR Count', browserStats.firCount ?? '-'],
    ['keyFrames', 'Keyframes Decoded', browserStats.keyFramesDecoded ?? '-'],
    ['codecStats', 'Browser Codec', browserStats.codecMimeType ? `${browserStats.codecMimeType} pt=${browserStats.codecPayloadType ?? '-'}` : '-'],
  ]);
  if (drone.ignored) modalMountedPlayer?.setStatus('ignored', 'status-warn');
  else if (lostMessage) {
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

async function loadState() { const response = await fetch('/api/drones'); render(await response.json()); }
function reconnectAll() { for (const player of players.values()) if (!player.drone.ignored && player.drone.mediaMtx?.ready) player.connect(); }

function renderTreeNode(label, value, options = {}) {
  const { open = false, hideLabel = false, path = label, state = null } = options;
  if (Array.isArray(value) || (value && typeof value === 'object')) {
    const entries = Array.isArray(value) ? value.map((child, index) => [String(index), child]) : Object.entries(value).filter(([, child]) => child !== undefined);
    const details = document.createElement('details');
    details.className = 'treeNode';
    details.dataset.treePath = path;
    details.open = state?.has(path) ? !!state.get(path) : open;
    details.addEventListener('toggle', () => state?.set(path, details.open));
    const summary = document.createElement('summary');
    summary.innerHTML = `${hideLabel ? '' : `<span class="treeKey">${escapeHtml(label)}</span>`}<span class="treeMeta">${Array.isArray(value) ? 'array' : 'object'}(${entries.length})</span>`;
    details.appendChild(summary);
    const children = document.createElement('div');
    children.className = 'treeChildren';
    entries.forEach(([childKey, childValue], index) => children.appendChild(renderTreeNode(childKey, childValue, { open: open && index < 3, path: `${path}.${childKey}`, state })));
    details.appendChild(children);
    return details;
  }
  const leaf = document.createElement('div');
  leaf.className = 'treeLeaf';
  const key = document.createElement('span');
  key.className = 'treeKey';
  key.textContent = label;
  const displayValue = formatTreeValue(value);
  const text = document.createElement('span');
  text.className = `treeValue ${displayValue.type}`;
  text.textContent = displayValue.text;
  leaf.append(key, text);
  return leaf;
}

function formatTreeValue(value) {
  if (value === null) return { text: 'null', type: 'null' };
  if (typeof value === 'boolean') return { text: value ? 'true' : 'false', type: 'boolean' };
  if (typeof value === 'number') return { text: Number.isFinite(value) ? String(value) : '-', type: 'number' };
  if (typeof value === 'string') return { text: value || '-', type: 'string' };
  return { text: String(value), type: 'string' };
}

function makePill(label, tone = '') {
  const element = document.createElement('span');
  element.className = `pill${tone ? ` ${tone}` : ''}`;
  element.textContent = label;
  return element;
}

function telemetryAgeLabel(drone) {
  return drone.lastTelemetryAt ? `${Math.max(0, Math.round((Date.now() - Date.parse(drone.lastTelemetryAt)) / 1000))}s ago` : 'no sample yet';
}

function magnitude(x, y, z) {
  const values = [Number(x), Number(y), Number(z)];
  if (values.some((value) => !Number.isFinite(value))) return NaN;
  return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
}

function msOrNull(seconds) {
  const value = Number(seconds);
  return Number.isFinite(value) ? Number((value * 1000).toFixed(1)) : null;
}

function escapeHtml(value) {
  return String(value ?? '').replaceAll('&', '&amp;').replaceAll('<', '&lt;').replaceAll('>', '&gt;').replaceAll('"', '&quot;').replaceAll("'", '&#39;');
}

document.querySelector('#discoverBtn').addEventListener('click', () => { fetch('/api/discover', { method: 'POST' }).catch(() => {}); });
document.querySelector('#reconnectBtn').addEventListener('click', reconnectAll);
modalCloseBtn.addEventListener('click', closeDroneModal);
droneModal.addEventListener('close', closeDroneModal);
droneModal.addEventListener('click', (event) => { if (event.target === droneModal) closeDroneModal(); });

document.querySelectorAll('.tabButton').forEach((button) => {
  button.addEventListener('click', () => {
    const tab = button.dataset.tab;
    document.querySelectorAll('.tabButton').forEach((item) => item.classList.toggle('active', item === button));
    document.querySelectorAll('.tabPanel').forEach((panel) => panel.classList.toggle('active', panel.dataset.panel === tab));
    if (tab === 'videoCharts' || tab === 'telemetryCharts') {
      updateCharts(tab);
      setTimeout(() => updateCharts(tab), 50);
    }
  });
});

const events = new EventSource('/api/events');
events.onmessage = (event) => {
  const message = JSON.parse(event.data);
  if (message.type === 'state') render(message.payload);
};

loadState();
setInterval(() => { if (latestState) render(latestState); }, 2000);