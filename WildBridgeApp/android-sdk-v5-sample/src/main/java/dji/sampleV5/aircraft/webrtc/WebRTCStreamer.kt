package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import org.webrtc.VideoCapturer
import java.net.Inet4Address
import java.net.NetworkInterface

/**
 * WebRTCStreamer manages DJI video capture and WHIP publishing for the drone feed.
 */
class WebRTCStreamer(
    context: Context,
    private val cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
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

    private val mainHandler = Handler(Looper.getMainLooper())
    private var sharedFrameSource: SharedDJIFrameSource? = null
    private var whipPublisher: WhipPublisher? = null
    @Volatile private var selectedOptions: WebRTCMediaOptions = options
    @Volatile private var currentOptions: WebRTCMediaOptions = optionsForSource(options, mockVideoEnabled)
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
        fun onMetrics(metrics: WebRTCStreamMetrics) {}
    }

    /**
     * Stop WHIP publishing and release capture resources.
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
        
        // Dispose shared frame source
        sharedFrameSource?.dispose()
        sharedFrameSource = null

        TelemetryProvider.stopListening()
        
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
        currentOptions = optionsForSource(selectedOptions, enabled)
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
    * Start publishing video via WHIP to a MediaMTX relay server. The phone
    * pushes its stream once and MediaMTX fans it out to WHEP consumers.
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

    fun isRunning(): Boolean = whipPublisher?.isRunning() == true

    /**
     * Change the streaming resolution for all active connections on-the-fly.
     */
    fun changeResolution(width: Int, height: Int) {
        Log.d(TAG, "Changing resolution to ${if (width > 0 && height > 0) "${width}x${height}" else "native"}")
        sharedFrameSource?.changeResolution(width, height)
        whipPublisher?.changeResolution(width, height)
    }

    /**
     * Store the new media defaults for future clients and apply resolution to
     * already-active WebRTC/WHIP capturers without reconnecting.
     */
    fun changeMediaOptions(options: WebRTCMediaOptions) {
        selectedOptions = options
        currentOptions = optionsForSource(options)
        desiredFps = currentOptions.fps.coerceIn(1, 60)
        effectiveFps = desiredFps
        saturationWindows = 0
        stableWindows = 0
        saturationState = "ok"
        changeResolution(currentOptions.videoResolutionWidth, currentOptions.videoResolutionHeight)
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
        Log.d(TAG, "Changing FPS to $fps: $reason")
        sharedFrameSource?.changeFrameRate(fps)
        whipPublisher?.changeFrameRate(fps)
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
            status = if (whipPublisher != null) metrics.status else "idle",
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

    private fun optionsForSource(
        baseOptions: WebRTCMediaOptions,
        mockEnabled: Boolean = useMockVideo
    ): WebRTCMediaOptions {
        if (!mockEnabled) return baseOptions

        // Mock MP4 playback cannot follow the "native" capture path because the
        // bundled asset needs an explicit output size. Keep the user's selected
        // DJI preset intact, but force the mock source onto a stable 1080p target.
        return WebRTCMediaOptions.fullHD().copy(
            fps = baseOptions.fps.coerceIn(1, 60),
            videoCodec = baseOptions.videoCodec
        )
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
