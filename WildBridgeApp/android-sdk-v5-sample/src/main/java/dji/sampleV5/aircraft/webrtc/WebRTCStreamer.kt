package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import org.json.JSONObject
import org.webrtc.VideoCapturer
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.ConcurrentHashMap

/**
 * WebRTCStreamer manages the WebRTC streaming server for the DJI drone video feed.
 * It handles the signaling server and manages all peer connections with viewers.
 */
class WebRTCStreamer(
    context: Context,
    private val cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val signalingPort: Int = 8081,
    private val droneName: String = "drone_1",
    private val options: WebRTCMediaOptions = WebRTCMediaOptions(),
    mockVideoEnabled: Boolean = false
) {

    companion object {
        private const val TAG = "WebRTCStreamer"
        private const val SATURATION_PROCESSING_RATIO = 0.75
        private const val SATURATION_WINDOWS_TO_THROTTLE = 2
        private const val STABLE_WINDOWS_TO_RELAX = 8
        private const val RECOVERY_COOLDOWN_MS = 15_000L
        private val ADAPTIVE_FPS_STEPS = intArrayOf(30, 25, 20, 15, 12, 10, 8, 6, 5)
    }

    private val appContext = context.applicationContext

    private var signalingServer: WebRTCSignalingServer? = null
    private val activeConnections = ConcurrentHashMap<String, WebRTCClient>()
    private val mainHandler = Handler(Looper.getMainLooper())
    private var sharedFrameSource: SharedDJIFrameSource? = null
    private var whipPublisher: WhipPublisher? = null
    @Volatile private var currentOptions: WebRTCMediaOptions = options
    @Volatile private var currentWhipUrl: String? = null
    @Volatile private var useMockVideo: Boolean = mockVideoEnabled
    private var badMetricsWindows = 0
    private var saturationWindows = 0
    private var stableWindows = 0
    private var recoveryCount = 0
    private var lastRecoveryAtMs = 0L
    private var desiredFps = options.fps.coerceIn(1, 60)
    private var effectiveFps = desiredFps
    @Volatile private var saturationState = "ok"
    
    var listener: WebRTCStreamerListener? = null

    interface WebRTCStreamerListener {
        fun onServerStarted(ip: String, port: Int)
        fun onServerStopped()
        fun onServerError(error: String)
        fun onClientConnected(clientId: String, totalClients: Int)
        fun onClientDisconnected(clientId: String, totalClients: Int)
        fun onMetrics(metrics: WebRTCStreamMetrics) {}
    }

    /**
     * Start the WebRTC streaming server
     */
    fun start() {
        Log.d(TAG, "Starting WebRTC streamer...")
        
        if (signalingServer != null) {
            Log.w(TAG, "Server already running")
            return
        }

        TelemetryProvider.startListening()
        
        signalingServer = WebRTCSignalingServer(signalingPort, droneName, object : WebRTCSignalingServer.SignalingServerListener {
            override fun onServerStarted(port: Int) {
                val ip = getLocalIpAddress() ?: "Unknown"
                Log.d(TAG, "Signaling server started at $ip:$port")
                mainHandler.post {
                    listener?.onServerStarted(ip, port)
                }
            }

            override fun onServerError(error: String) {
                Log.e(TAG, "Signaling server error: $error")
                mainHandler.post {
                    listener?.onServerError(error)
                }
            }

            override fun onClientConnected(clientId: String) {
                Log.d(TAG, "Client connected: $clientId")
                createPeerConnection(clientId)
            }

            override fun onClientDisconnected(clientId: String) {
                Log.d(TAG, "Client disconnected: $clientId")
                removePeerConnection(clientId)
            }

            override fun onWebRTCMessage(clientId: String, message: JSONObject) {
                Log.d(TAG, "WebRTC message from $clientId: ${message.optString("type")}")
                handleWebRTCMessage(clientId, message)
            }
        })
        
        try {
            signalingServer?.start()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start signaling server: ${e.message}", e)
            listener?.onServerError("Failed to start server: ${e.message}")
        }
    }

    /**
     * Stop the WebRTC streaming server and all connections.
     */
    fun stop() {
        Log.d(TAG, "Stopping WebRTC streamer...")
        mainHandler.removeCallbacksAndMessages(null)
        val stoppedListener = listener
        listener = null
        currentWhipUrl = null
        badMetricsWindows = 0

        logWhipLifecycle(
            event = "whip_stop_requested",
            detail = "streamer stop invoked"
        )
        
        // Stop WHIP publisher if active
        whipPublisher?.stop()
        whipPublisher = null
        
        // Close all peer connections
        activeConnections.values.forEach { client ->
            client.dispose()
        }
        activeConnections.clear()
        
        // Dispose shared frame source
        sharedFrameSource?.dispose()
        sharedFrameSource = null

        TelemetryProvider.stopListening()
        
        // Stop signaling server
        signalingServer?.stopServer()
        signalingServer = null
        
        if (Looper.myLooper() == Looper.getMainLooper()) {
            stoppedListener?.onServerStopped()
        } else {
            mainHandler.post {
                stoppedListener?.onServerStopped()
            }
        }
        
        Log.d(TAG, "WebRTC streamer stopped")
    }

    fun setMockVideoEnabled(enabled: Boolean) {
        if (useMockVideo == enabled) return
        val previousSource = if (useMockVideo) "mock" else "dji"
        useMockVideo = enabled
        badMetricsWindows = 0
        saturationWindows = 0
        stableWindows = 0
        saturationState = "ok"
        recoveryCount = 0

        logWhipLifecycle(
            event = "whip_source_switched",
            detail = "source $previousSource -> ${if (enabled) "mock" else "dji"}"
        )

        if (enabled) {
            sharedFrameSource?.dispose()
            sharedFrameSource = null
        }

        activeConnections.values.forEach { it.dispose() }
        activeConnections.clear()

        val whipUrl = currentWhipUrl
        whipPublisher?.stop()
        whipPublisher = null
        if (whipUrl != null) {
            mainHandler.postDelayed({
                if (currentWhipUrl == whipUrl) startWhip(whipUrl)
            }, 500L)
        }
        Log.i(TAG, "Video source changed to ${if (enabled) "mock MP4" else "DJI camera"}")
    }

    fun isMockVideoEnabled(): Boolean = useMockVideo

    /**
     * Start publishing video via WHIP to a mediamtx relay server.
     * This replaces the signaling server approach — the phone pushes
     * its stream once and mediamtx fans it out to all consumers.
     *
     * @param whipUrl Full WHIP endpoint URL, e.g. "http://192.168.x.y:8889/drone_1/whip"
     */
    fun startWhip(whipUrl: String) {
        Log.d(TAG, "Starting WHIP publisher to $whipUrl")
        logWhipLifecycle(
            event = "whip_start_requested",
            whipUrl = whipUrl,
            detail = "fps=$effectiveFps target=${resolutionLabelForOptions(currentOptions)}"
        )

        whipPublisher?.let { existingPublisher ->
            if (currentWhipUrl == whipUrl && existingPublisher.isRunning() && existingPublisher.isPublishing()) {
                Log.d(TAG, "WHIP publisher already healthy for $whipUrl")
                logWhipLifecycle(
                    event = "whip_start_skipped",
                    whipUrl = whipUrl,
                    detail = "publisher already running"
                )
                return
            }
            Log.w(TAG, "Restarting stale WHIP publisher for $whipUrl")
            logWhipLifecycle(
                event = "whip_restart_requested",
                whipUrl = whipUrl,
                detail = "stale publisher detected"
            )
            existingPublisher.stop()
            whipPublisher = null
        }

        TelemetryProvider.startListening()
        currentWhipUrl = whipUrl

        val djiFrameSource = if (useMockVideo) null else getOrCreateSharedSource()
        val startFrameCount = djiFrameSource?.totalOutputFrames() ?: 0L
        val capturer = createVideoCapturer("whip")

        whipPublisher = WhipPublisher(
            context = appContext,
            videoCapturer = capturer,
            options = currentOptions,
            whipUrl = whipUrl
        ).apply {
            this.listener = object : WhipPublisher.WhipListener {
                override fun onPublishing() {
                    val ip = getLocalIpAddress() ?: "Unknown"
                    Log.i(TAG, "WHIP publishing from $ip to $whipUrl")
                    logWhipLifecycle(
                        event = "whip_publishing",
                        whipUrl = whipUrl,
                        detail = "localIp=$ip"
                    )
                    mainHandler.post {
                        this@WebRTCStreamer.listener?.onServerStarted(ip, 0)
                    }
                }
                override fun onDisconnected() {
                    Log.w(TAG, "WHIP connection lost")
                    logWhipLifecycle(
                        event = "whip_disconnected",
                        whipUrl = whipUrl,
                        detail = "publisher disconnected"
                    )
                }
                override fun onError(error: String) {
                    Log.e(TAG, "WHIP error: $error")
                    logWhipLifecycle(
                        event = "whip_error",
                        whipUrl = whipUrl,
                        detail = error
                    )
                    mainHandler.post {
                        this@WebRTCStreamer.listener?.onServerError("WHIP: $error")
                    }
                }
            }
            start()
        }

        mainHandler.postDelayed({
            val noFramesSinceStart = !useMockVideo && djiFrameSource != null && djiFrameSource.observerCount() > 0 && djiFrameSource.totalOutputFrames() == startFrameCount
            if (currentWhipUrl == whipUrl && noFramesSinceStart) {
                val message = "Camera feed lost. The drone may have been idle too long or overheated; power-cycle the drone and let it cool down before retrying."
                Log.w(TAG, message)
                logWhipLifecycle(
                    event = "whip_source_lost",
                    whipUrl = whipUrl,
                    detail = "no new DJI frames after start"
                )
                mainHandler.post { listener?.onServerError(message) }
            }
        }, 15_000L)
    }

    /**
     * Check if the server is running
     */
    fun isRunning(): Boolean = signalingServer != null

    /**
     * Get the number of connected clients
     */
    fun getClientCount(): Int = activeConnections.size

    /**
     * Change the streaming resolution for all active connections on-the-fly.
     */
    fun changeResolution(width: Int, height: Int) {
        Log.d(TAG, "Changing resolution to ${if (width > 0 && height > 0) "${width}x${height}" else "native"} for ${activeConnections.size} client(s)")
        sharedFrameSource?.changeResolution(width, height)
        whipPublisher?.changeResolution(width, height)
        activeConnections.values.forEach { client ->
            client.changeResolution(width, height)
        }
    }

    /**
     * Store the new media defaults for future clients and apply resolution to
     * already-active WebRTC/WHIP capturers without reconnecting.
     */
    fun changeMediaOptions(options: WebRTCMediaOptions) {
        currentOptions = options
        desiredFps = options.fps.coerceIn(1, 60)
        effectiveFps = desiredFps
        saturationWindows = 0
        stableWindows = 0
        saturationState = "ok"
        changeResolution(options.videoResolutionWidth, options.videoResolutionHeight)
        applyFrameRate(effectiveFps, "media options updated")
    }

    fun changeFrameRate(fps: Int) {
        val boundedFps = fps.coerceIn(1, 60)
        desiredFps = boundedFps
        effectiveFps = boundedFps
        saturationWindows = 0
        stableWindows = 0
        saturationState = "ok"
        applyFrameRate(boundedFps, "manual change")
    }

    private fun applyFrameRate(fps: Int, reason: String) {
        Log.d(TAG, "Changing FPS to $fps for ${activeConnections.size} client(s): $reason")
        sharedFrameSource?.changeFrameRate(fps)
        whipPublisher?.changeFrameRate(fps)
        activeConnections.values.forEach { client ->
            client.changeFrameRate(fps)
        }
    }

    /**
     * Get the connection URL for clients
     */
    fun getConnectionUrl(): String {
        val ip = getLocalIpAddress() ?: "Unknown"
        return "ws://$ip:$signalingPort"
    }

    private fun getOrCreateSharedSource(): SharedDJIFrameSource {
        return sharedFrameSource ?: SharedDJIFrameSource(cameraIndex, droneName).also {
            it.metricsListener = ::handleFrameSourceMetrics
            sharedFrameSource = it
        }
    }

    private fun createVideoCapturer(clientId: String): VideoCapturer {
        return if (useMockVideo) {
            MockMp4VideoCapturer(droneName).apply {
                metricsListener = ::handleFrameSourceMetrics
            }
        } else {
            SharedVideoCapturerHandle(clientId, getOrCreateSharedSource())
        }
    }

    private fun handleFrameSourceMetrics(metrics: WebRTCStreamMetrics) {
        maybeAdaptFrameRate(metrics)
        val enriched = metrics.copy(
            recoveryCount = recoveryCount,
            status = if (whipPublisher != null || signalingServer != null) metrics.status else "idle",
            configuredFps = desiredFps,
            saturationState = saturationState,
            scaleMode = if (currentOptions.usesSourceResolution) "native" else "fixed"
        )
        if (!useMockVideo) maybeRecoverStreaming(enriched)
        mainHandler.post { listener?.onMetrics(enriched) }
    }

    private fun maybeAdaptFrameRate(metrics: WebRTCStreamMetrics) {
        if (metrics.observerCount == 0 || metrics.targetFps <= 0) {
            saturationWindows = 0
            stableWindows = 0
            saturationState = "ok"
            return
        }

        val frameBudgetMs = 1000.0 / metrics.targetFps.toDouble()
        val processingSaturated = metrics.averageFrameProcessingMs >= frameBudgetMs * SATURATION_PROCESSING_RATIO
        val sourceFlowing = metrics.inputFps >= maxOf(2.0, metrics.targetFps * 0.6)
        val outputLagging = sourceFlowing && metrics.outputFps < metrics.targetFps * 0.5
        val processingErrorsActive = metrics.processingErrors > 0
        val saturated = processingSaturated || (outputLagging && processingErrorsActive)

        if (saturated) {
            saturationWindows += 1
            stableWindows = 0
            saturationState = if (processingSaturated) "hot" else "error"
            if (saturationWindows >= SATURATION_WINDOWS_TO_THROTTLE) {
                val nextFps = nextLowerAdaptiveFps(effectiveFps)
                if (nextFps < effectiveFps) {
                    effectiveFps = nextFps
                    saturationWindows = 0
                    logWhipLifecycle(
                        event = "adaptive_fps_lowered",
                        detail = "fps ${metrics.targetFps} -> $effectiveFps proc=${metrics.averageFrameProcessingMs.format1()}ms budget=${frameBudgetMs.format1()}ms out=${metrics.outputFps.format1()} in=${metrics.inputFps.format1()} err=${metrics.processingErrors}"
                    )
                    applyFrameRate(effectiveFps, "processing saturation")
                }
            }
            return
        }

        if (outputLagging && effectiveFps == desiredFps) {
            saturationWindows = 0
            stableWindows = 0
            saturationState = "source-limited"
            return
        }

        if (effectiveFps < desiredFps) {
            stableWindows += 1
            saturationState = "recovering"
            if (stableWindows >= STABLE_WINDOWS_TO_RELAX) {
                val nextFps = nextHigherAdaptiveFps(effectiveFps, desiredFps)
                if (nextFps > effectiveFps) {
                    effectiveFps = nextFps
                    stableWindows = 0
                    logWhipLifecycle(
                        event = "adaptive_fps_raised",
                        detail = "fps ${metrics.targetFps} -> $effectiveFps proc=${metrics.averageFrameProcessingMs.format1()}ms out=${metrics.outputFps.format1()} in=${metrics.inputFps.format1()}"
                    )
                    applyFrameRate(effectiveFps, "saturation recovered")
                }
            }
        } else {
            stableWindows = 0
            saturationState = "ok"
        }

        if (effectiveFps == desiredFps && !saturated) {
            saturationState = "ok"
        }
    }

    private fun maybeRecoverStreaming(metrics: WebRTCStreamMetrics) {
        if (metrics.observerCount == 0) {
            badMetricsWindows = 0
            return
        }

        val stalled = metrics.inputFps > 1.0 && metrics.outputFps < 1.0
        val erroring = metrics.processingErrors > 0 && metrics.outputFps < metrics.targetFps * 0.25
        badMetricsWindows = if (stalled || erroring) badMetricsWindows + 1 else 0

        val now = System.currentTimeMillis()
        if (badMetricsWindows >= 3 && now - lastRecoveryAtMs > RECOVERY_COOLDOWN_MS) {
            lastRecoveryAtMs = now
            recoveryCount++
            badMetricsWindows = 0
            val reason = "low output fps ${metrics.outputFps} with input fps ${metrics.inputFps}"
            Log.w(TAG, "Recovering WebRTC pipeline: $reason")
            logWhipLifecycle(
                event = "whip_source_degraded",
                detail = "$reason proc=${metrics.averageFrameProcessingMs}ms req=${metrics.requestedWidth}x${metrics.requestedHeight} src=${metrics.sourceWidth}x${metrics.sourceHeight}"
            )
            sharedFrameSource?.recoverCapture(reason)
            restartWhipPublisher(reason)
        }
    }

    private fun restartWhipPublisher(reason: String) {
        val whipUrl = currentWhipUrl ?: return
        val oldPublisher = whipPublisher ?: return
        Log.w(TAG, "Restarting WHIP publisher: $reason")
        logWhipLifecycle(
            event = "whip_restart_requested",
            whipUrl = whipUrl,
            detail = reason
        )
        oldPublisher.stop()
        whipPublisher = null
        mainHandler.postDelayed({
            if (currentWhipUrl == whipUrl) {
                startWhip(whipUrl)
            }
        }, 1000L)
    }

    private fun createPeerConnection(clientId: String) {
        Log.d(TAG, "Creating peer connection for: $clientId")
        
        // Create a lightweight handle backed by the shared frame source
        val videoCapturer = createVideoCapturer(clientId)
        
        val client = WebRTCClient(
            clientId = clientId,
            context = appContext,
            videoCapturer = videoCapturer,
            options = currentOptions,
            messageCallback = { id, message ->
                signalingServer?.sendToClient(id, message)
            }
        )
        
        // Note: WebRTCClient internally sets up metadata listener on the capturer
        
        client.connectionListener = object : WebRTCClient.PeerConnectionListener {
            override fun onConnected(clientId: String) {
                Log.d(TAG, "Peer connected: $clientId")
                mainHandler.post {
                    listener?.onClientConnected(clientId, activeConnections.size)
                }
            }

            override fun onDisconnected(clientId: String) {
                Log.d(TAG, "Peer disconnected: $clientId")
                mainHandler.post {
                    removePeerConnection(clientId)
                }
            }

            override fun onError(clientId: String, error: String) {
                Log.e(TAG, "Peer error for $clientId: $error")
            }
        }
        
        activeConnections[clientId] = client
        
        // Send offer to the client to initiate the connection
        client.createOffer()
        
        mainHandler.post {
            listener?.onClientConnected(clientId, activeConnections.size)
        }
    }

    private fun removePeerConnection(clientId: String) {
        activeConnections.remove(clientId)?.let { client ->
            client.dispose()
            mainHandler.post {
                listener?.onClientDisconnected(clientId, activeConnections.size)
            }
        }
    }

    private fun handleWebRTCMessage(clientId: String, message: JSONObject) {
        activeConnections[clientId]?.handleSignalingMessage(message)
            ?: Log.w(TAG, "No connection found for client: $clientId")
    }

    /**
     * Get the local IP address of the device
     */
    fun getLocalIpAddress(): String? {
        try {
            val interfaces = NetworkInterface.getNetworkInterfaces()
            while (interfaces.hasMoreElements()) {
                val networkInterface = interfaces.nextElement()
                val addresses = networkInterface.inetAddresses
                while (addresses.hasMoreElements()) {
                    val address = addresses.nextElement()
                    if (!address.isLoopbackAddress && address is Inet4Address) {
                        return address.hostAddress
                    }
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error getting local IP: ${e.message}", e)
        }
        return null
    }

    private fun logWhipLifecycle(
        event: String,
        whipUrl: String? = currentWhipUrl,
        detail: String? = null
    ) {
        val sharedSource = sharedFrameSource
        val frameCount = sharedSource?.totalOutputFrames() ?: 0L
        val observers = sharedSource?.observerCount() ?: 0
        val source = if (useMockVideo) "mock" else "dji"
        val suffix = buildString {
            append("event=")
            append(event)
            append(" source=")
            append(source)
            append(" whipUrl=")
            append(whipUrl ?: "none")
            append(" frames=")
            append(frameCount)
            append(" observers=")
            append(observers)
            detail?.takeIf { it.isNotBlank() }?.let {
                append(" detail=")
                append(it)
            }
        }
        Log.i(TAG, "WHIP lifecycle $suffix")
    }

    private fun nextLowerAdaptiveFps(current: Int): Int {
        val currentIndex = ADAPTIVE_FPS_STEPS.indexOfFirst { it == current }
        if (currentIndex >= 0 && currentIndex < ADAPTIVE_FPS_STEPS.lastIndex) {
            return ADAPTIVE_FPS_STEPS[currentIndex + 1]
        }
        return ADAPTIVE_FPS_STEPS.firstOrNull { it < current } ?: current
    }

    private fun nextHigherAdaptiveFps(current: Int, desired: Int): Int {
        val currentIndex = ADAPTIVE_FPS_STEPS.indexOfFirst { it == current }
        if (currentIndex > 0) {
            return ADAPTIVE_FPS_STEPS[currentIndex - 1].coerceAtMost(desired)
        }
        return desired
    }

    private fun resolutionLabelForOptions(options: WebRTCMediaOptions): String {
        return if (options.usesSourceResolution) {
            "native"
        } else {
            "${options.videoResolutionWidth}x${options.videoResolutionHeight}"
        }
    }

    private fun Double.format1(): String = String.format(java.util.Locale.US, "%.1f", this)
}
