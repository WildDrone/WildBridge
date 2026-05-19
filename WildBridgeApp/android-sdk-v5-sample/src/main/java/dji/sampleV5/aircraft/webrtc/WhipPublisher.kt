package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import org.webrtc.*
import java.io.OutputStreamWriter
import java.net.HttpURLConnection
import java.net.URL
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Publishes a WebRTC video stream to a mediamtx server via WHIP
 * (WebRTC HTTP Ingest Protocol).
 *
 * Flow:
 *  1. Create PeerConnection with a sendonly video track
 *  2. Create SDP offer and gather all ICE candidates
 *  3. POST the offer to the WHIP endpoint
 *  4. Set the SDP answer from the response
 *  5. Video flows through mediamtx to all WHEP consumers
 *
 * Reconnects automatically if the connection drops.
 */
class WhipPublisher(
    context: Context,
    private val videoCapturer: VideoCapturer,
    private val options: WebRTCMediaOptions = WebRTCMediaOptions(),
    private val whipUrl: String
) {
    companion object {
        private const val TAG = "WhipPublisher"
        private const val ICE_GATHER_TIMEOUT_S = 10L
        private const val RECONNECT_BASE_DELAY_MS = 2000L
        private const val RECONNECT_MAX_DELAY_MS = 30000L
        private const val FIRST_FRAME_TIMEOUT_MS = 8_000L
        private const val FIRST_FRAME_RECOVERY_TIMEOUT_MS = 4_000L
    }

    private val appContext = context.applicationContext

    private val executor = Executors.newSingleThreadExecutor()
    private val mainHandler = Handler(Looper.getMainLooper())

    private var peerConnection: PeerConnection? = null
    private var videoSource: VideoSource? = null
    private var videoTrack: VideoTrack? = null
    private var whipResourceUrl: String? = null  // Location header for DELETE on teardown

    private val isRunning = AtomicBoolean(false)
    private val isPublishing = AtomicBoolean(false)
    private val isTearingDown = AtomicBoolean(false)
    @Volatile private var currentFps: Int = options.fps

    var listener: WhipListener? = null

    interface WhipListener {
        fun onPublishing()
        fun onDisconnected()
        fun onError(error: String)
    }

    fun start() {
        if (isRunning.getAndSet(true)) return
        executor.execute { publishLoop() }
    }

    fun isRunning(): Boolean = isRunning.get()

    fun isPublishing(): Boolean = isPublishing.get()

    fun stop() {
        val wasRunning = isRunning.getAndSet(false)
        listener = null
        mainHandler.removeCallbacksAndMessages(null)
        executor.shutdownNow()
        if (wasRunning) {
            teardown()
            Log.i(TAG, "WhipPublisher stopped")
        }
    }

    /** Change resolution without reconnection. */
    fun changeResolution(width: Int, height: Int) {
        when (videoCapturer) {
            is DJIV5VideoCapturer -> videoCapturer.changeResolution(width, height)
            is SharedVideoCapturerHandle -> videoCapturer.changeResolution(width, height)
            is MockMp4VideoCapturer -> videoCapturer.changeResolution(width, height)
        }
    }

    fun changeFrameRate(fps: Int) {
        val boundedFps = fps.coerceIn(1, 60)
        currentFps = boundedFps
        when (videoCapturer) {
            is DJIV5VideoCapturer -> videoCapturer.changeCaptureFormat(options.videoResolutionWidth, options.videoResolutionHeight, boundedFps)
            is SharedVideoCapturerHandle -> videoCapturer.changeFrameRate(boundedFps)
            is MockMp4VideoCapturer -> videoCapturer.changeFrameRate(boundedFps)
        }
        peerConnection?.senders?.firstOrNull()?.let { configureVideoSenderForStability(it) }
        Log.d(TAG, "WHIP frame rate changed to $boundedFps fps")
    }

    // ── internal ────────────────────────────────────────────────────

    private fun publishLoop() {
        var consecutiveFailures = 0

        while (isRunning.get()) {
            try {
                publish()
                // publish() blocks until disconnection
                consecutiveFailures = 0
            } catch (e: Exception) {
                if (!isRunning.get()) break
                consecutiveFailures++
                Log.e(TAG, "Publish failed (attempt $consecutiveFailures): ${e.message}")
                mainHandler.post { listener?.onError(e.message ?: "Unknown error") }
            } finally {
                teardown()
                isPublishing.set(false)
                mainHandler.post { listener?.onDisconnected() }
            }

            if (!isRunning.get()) break

            val delay = (RECONNECT_BASE_DELAY_MS * (1L shl minOf(consecutiveFailures - 1, 4)))
                .coerceAtMost(RECONNECT_MAX_DELAY_MS)
            Log.i(TAG, "Reconnecting in ${delay}ms...")
            try {
                Thread.sleep(delay)
            } catch (_: InterruptedException) {
                Thread.currentThread().interrupt()
                break
            }
        }
    }

    /**
     * Single publish attempt. Blocks until the connection closes or
     * [isRunning] becomes false.
     */
    private fun publish() {
        Log.i(TAG, "Publishing to $whipUrl")

        val factory = WebRTCClient.getFactory(appContext)

        // 1. Create video source & track
        videoSource = factory.createVideoSource(false)
        val startingFrameCount = when (videoCapturer) {
            is SharedVideoCapturerHandle -> videoCapturer.totalOutputFrames()
            is MockMp4VideoCapturer -> videoCapturer.totalOutputFrames()
            else -> 0L
        }
        videoCapturer.initialize(null, appContext, videoSource!!.capturerObserver)
        videoCapturer.startCapture(
            options.videoResolutionWidth,
            options.videoResolutionHeight,
            currentFps
        )

        when (videoCapturer) {
            is SharedVideoCapturerHandle -> {
                if (!videoCapturer.waitForOutputFrameAfter(startingFrameCount, FIRST_FRAME_TIMEOUT_MS)) {
                    Log.w(TAG, "No DJI video frames before WHIP offer; recovering capture")
                    videoCapturer.recoverCapture("no frames before WHIP offer")
                    if (!videoCapturer.waitForOutputFrameAfter(startingFrameCount, FIRST_FRAME_RECOVERY_TIMEOUT_MS)) {
                        throw IllegalStateException("No DJI video frames available for WHIP publishing")
                    }
                }
            }
            is MockMp4VideoCapturer -> {
                if (!videoCapturer.waitForOutputFrameAfter(startingFrameCount, FIRST_FRAME_TIMEOUT_MS)) {
                    throw IllegalStateException("No mock MP4 video frames available for WHIP publishing")
                }
            }
        }

        videoTrack = factory.createVideoTrack(options.videoTrackId, videoSource).apply {
            setEnabled(true)
        }

        // 2. Create PeerConnection
        val rtcConfig = PeerConnection.RTCConfiguration(
            listOf(
                PeerConnection.IceServer.builder("stun:stun.l.google.com:19302")
                    .createIceServer()
            )
        ).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
        }

        // Disable CPU overuse detection so WebRTC doesn't auto-downscale resolution
        disableCpuOveruseDetection(rtcConfig)

        val iceGatherLatch = CountDownLatch(1)
        val connected = AtomicBoolean(false)

        peerConnection = factory.createPeerConnection(rtcConfig, object : PeerConnection.Observer {
            override fun onSignalingChange(s: PeerConnection.SignalingState) {}
            override fun onIceConnectionChange(s: PeerConnection.IceConnectionState) {
                Log.d(TAG, "ICE connection: $s")
                when (s) {
                    PeerConnection.IceConnectionState.CONNECTED -> {
                        connected.set(true)
                        isPublishing.set(true)
                        mainHandler.post { listener?.onPublishing() }
                    }
                    PeerConnection.IceConnectionState.FAILED,
                    PeerConnection.IceConnectionState.DISCONNECTED,
                    PeerConnection.IceConnectionState.CLOSED -> {
                        connected.set(false)
                    }
                    else -> {}
                }
            }
            override fun onIceConnectionReceivingChange(b: Boolean) {}
            override fun onIceGatheringChange(s: PeerConnection.IceGatheringState) {
                if (s == PeerConnection.IceGatheringState.COMPLETE) {
                    iceGatherLatch.countDown()
                }
            }
            override fun onIceCandidate(c: IceCandidate) {}
            override fun onIceCandidatesRemoved(c: Array<out IceCandidate>) {}
            override fun onAddStream(s: MediaStream) {}
            override fun onRemoveStream(s: MediaStream) {}
            override fun onDataChannel(dc: DataChannel) {}
            override fun onRenegotiationNeeded() {}
            override fun onAddTrack(r: RtpReceiver, ss: Array<out MediaStream>) {}
        })

        // Add video track (sendonly — mediamtx doesn't send back video)
        peerConnection!!.addTrack(videoTrack, listOf(options.mediaStreamId))

        // Configure sender for stable resolution — match WebRTCClient tuning
        peerConnection!!.senders.firstOrNull()?.let { sender ->
            configureVideoSenderForStability(sender)
        }

        // 3. Create offer
        val offerLatch = CountDownLatch(1)
        var localSdp: SessionDescription? = null

        val constraints = MediaConstraints().apply {
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveVideo", "false"))
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveAudio", "false"))
        }

        peerConnection!!.createOffer(object : SdpObserver {
            override fun onCreateSuccess(sdp: SessionDescription) {
                // Force H264 and set short keyframe interval for loss recovery
                val mungedSdp = SessionDescription(
                    sdp.type,
                    SdpUtils.mungeForH264(sdp.description)
                )
                peerConnection!!.setLocalDescription(object : SdpObserver {
                    override fun onSetSuccess() {
                        localSdp = mungedSdp
                        offerLatch.countDown()
                    }
                    override fun onSetFailure(err: String) {
                        Log.e(TAG, "setLocalDescription failed: $err")
                        offerLatch.countDown()
                    }
                    override fun onCreateSuccess(s: SessionDescription?) {}
                    override fun onCreateFailure(s: String?) {}
                }, mungedSdp)
            }
            override fun onCreateFailure(err: String) {
                Log.e(TAG, "createOffer failed: $err")
                offerLatch.countDown()
            }
            override fun onSetSuccess() {}
            override fun onSetFailure(s: String?) {}
        }, constraints)

        offerLatch.await(5, TimeUnit.SECONDS)
        if (localSdp == null) throw IllegalStateException("Failed to create SDP offer")

        // 4. Wait for ICE gathering to finish (full SDP needed for WHIP)
        if (!iceGatherLatch.await(ICE_GATHER_TIMEOUT_S, TimeUnit.SECONDS)) {
            Log.w(TAG, "ICE gathering timeout — proceeding with partial candidates")
        }

        // Use the local description which now contains all gathered ICE candidates
        val offerSdp = peerConnection!!.localDescription?.description
            ?: throw IllegalStateException("No local description after ICE gathering")

        // 5. POST offer to WHIP endpoint
        val answerSdp = postWhipOffer(offerSdp)

        // 6. Set remote description (answer from mediamtx)
        val answerLatch = CountDownLatch(1)
        val answer = SessionDescription(SessionDescription.Type.ANSWER, answerSdp)
        peerConnection!!.setRemoteDescription(object : SdpObserver {
            override fun onSetSuccess() { answerLatch.countDown() }
            override fun onSetFailure(err: String) {
                Log.e(TAG, "setRemoteDescription failed: $err")
                answerLatch.countDown()
            }
            override fun onCreateSuccess(s: SessionDescription?) {}
            override fun onCreateFailure(s: String?) {}
        }, answer)
        answerLatch.await(5, TimeUnit.SECONDS)

        Log.i(TAG, "WHIP publish started — waiting for connection")

        // 7. Wait until connection drops or we're stopped
        while (isRunning.get() && (connected.get() || peerConnection?.iceConnectionState() == PeerConnection.IceConnectionState.CHECKING)) {
            Thread.sleep(500)
        }

        if (isRunning.get()) {
            Log.w(TAG, "WHIP connection lost — will reconnect")
        }
    }

    /**
     * HTTP POST of SDP offer to the WHIP endpoint.
     * Returns the SDP answer body.
     */
    private fun postWhipOffer(offerSdp: String): String {
        val url = URL(whipUrl)
        val conn = url.openConnection() as HttpURLConnection
        try {
            conn.requestMethod = "POST"
            conn.setRequestProperty("Content-Type", "application/sdp")
            conn.doOutput = true
            conn.connectTimeout = 10_000
            conn.readTimeout = 10_000

            OutputStreamWriter(conn.outputStream, Charsets.UTF_8).use { it.write(offerSdp) }

            val status = conn.responseCode
            if (status == 201) {
                val location = conn.getHeaderField("Location")
                if (location != null) {
                    // Make absolute if relative
                    whipResourceUrl = if (location.startsWith("http")) location
                        else "${url.protocol}://${url.host}:${url.port}$location"
                }
                return conn.inputStream.bufferedReader().readText()
            } else {
                val body = runCatching { conn.errorStream?.bufferedReader()?.readText() }.getOrNull() ?: ""
                throw RuntimeException("WHIP POST failed: $status $body")
            }
        } finally {
            conn.disconnect()
        }
    }

    private fun deleteWhipResource() {
        val resourceUrl = whipResourceUrl ?: return
        whipResourceUrl = null
        try {
            val conn = URL(resourceUrl).openConnection() as HttpURLConnection
            conn.requestMethod = "DELETE"
            conn.connectTimeout = 3000
            conn.readTimeout = 3000
            val status = conn.responseCode
            conn.disconnect()
            Log.d(TAG, "WHIP resource DELETE: $status")
        } catch (e: Exception) {
            Log.d(TAG, "WHIP resource DELETE failed: ${e.message}")
        }
    }

    private fun teardown() {
        if (!isTearingDown.compareAndSet(false, true)) return
        try {
            deleteWhipResource()
            runCatching { videoCapturer.stopCapture() }
            runCatching { videoTrack?.dispose() }
            videoTrack = null
            runCatching { videoSource?.dispose() }
            videoSource = null
            runCatching { peerConnection?.dispose() }
                .onFailure { Log.d(TAG, "PeerConnection dispose ignored: ${it.message}") }
            peerConnection = null
        } finally {
            isTearingDown.set(false)
        }
    }

    /**
     * Disable WebRTC's internal CPU overuse detector which auto-downscales
     * resolution when it thinks the device is under load.
     */
    private fun disableCpuOveruseDetection(rtcConfig: PeerConnection.RTCConfiguration) {
        runCatching {
            val field = rtcConfig.javaClass.getField("enableCpuOveruseDetection")
            field.isAccessible = true
            field.setBoolean(rtcConfig, false)
            Log.d(TAG, "Disabled RTC CPU overuse detection")
        }.onFailure {
            Log.d(TAG, "RTC CPU overuse flag unavailable on this WebRTC build")
        }
    }

    /**
    * Configure the RTP sender to maintain framerate under load:
     * - Set max bitrate and framerate
    * - Set DegradationPreference to MAINTAIN_FRAMERATE (scale before dropping FPS)
     */
    private fun configureVideoSenderForStability(sender: RtpSender) {
        runCatching {
            val params = sender.parameters ?: return
            val encodings = params.encodings ?: emptyList()
            val bitrateCap = options.senderBitrateBps()

            encodings.forEach { encoding ->
                runCatching { encoding.maxBitrateBps = bitrateCap }
                runCatching { encoding.maxFramerate = currentFps }
            }

            // Force adaptation strategy toward FPS reduction before resolution reduction
            runCatching {
                val preferenceClass = Class.forName("org.webrtc.RtpParameters\$DegradationPreference")
                @Suppress("UNCHECKED_CAST")
                val enumClass = preferenceClass as Class<out Enum<*>>
                val maintainFramerate = java.lang.Enum.valueOf(enumClass, "MAINTAIN_FRAMERATE")
                val field = params.javaClass.getField("degradationPreference")
                field.isAccessible = true
                field.set(params, maintainFramerate)
            }

            sender.parameters = params
            Log.d(TAG, "Sender params tuned: maxBitrate=${bitrateCap}bps, maxFps=$currentFps, prefer=MAINTAIN_FRAMERATE")
        }.onFailure { e ->
            Log.w(TAG, "Unable to fully apply sender tuning: ${e.message}")
        }
    }
}
