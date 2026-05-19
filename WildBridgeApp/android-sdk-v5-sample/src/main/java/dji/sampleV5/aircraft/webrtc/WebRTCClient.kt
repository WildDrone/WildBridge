package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.util.Log
import org.json.JSONException
import org.json.JSONObject
import org.webrtc.*
import java.nio.ByteBuffer
import java.nio.charset.StandardCharsets
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Manages a single WebRTC peer connection with a remote client.
 * Handles SDP negotiation, ICE candidates, and video streaming.
 * Also manages a data channel for synchronized frame metadata.
 */
class WebRTCClient(
    val clientId: String,
    context: Context,
    private val videoCapturer: VideoCapturer,
    private val options: WebRTCMediaOptions = WebRTCMediaOptions(),
    private val messageCallback: (String, JSONObject) -> Unit
) {

    companion object {
        private const val TAG = "WebRTCClient"
        private const val METADATA_CHANNEL_LABEL = "telemetry"
        private const val METADATA_BUFFER_THRESHOLD = 64 * 1024L  // 64 KB
        
        @Volatile
        private var factory: PeerConnectionFactory? = null
        private var eglBase: EglBase? = null
        private val factoryLock = Any()
        
        fun getEglBase(): EglBase {
            synchronized(factoryLock) {
                if (eglBase == null) {
                    eglBase = EglBase.create()
                }
                return eglBase!!
            }
        }
        
        fun getFactory(context: Context): PeerConnectionFactory {
            synchronized(factoryLock) {
                if (factory == null) {
                    initializeFactory(context)
                }
                return factory!!
            }
        }
        
        private fun initializeFactory(context: Context) {
            val initOptions = PeerConnectionFactory.InitializationOptions.builder(context)
                .setEnableInternalTracer(true)
                .setFieldTrials("WebRTC-H264HighProfile/Enabled/")
                .createInitializationOptions()
            PeerConnectionFactory.initialize(initOptions)
            
            val rootEglBase = getEglBase()
            
            factory = PeerConnectionFactory.builder()
                .setVideoDecoderFactory(DefaultVideoDecoderFactory(rootEglBase.eglBaseContext))
                .setVideoEncoderFactory(DefaultVideoEncoderFactory(
                    rootEglBase.eglBaseContext, 
                    false, // enableIntelVp8Encoder — unused, avoid probing overhead
                    true   // enableH264HighProfile
                ))
                .setOptions(PeerConnectionFactory.Options())
                .createPeerConnectionFactory()
            
            Log.d(TAG, "PeerConnectionFactory initialized")
        }
    }

    private var peerConnection: PeerConnection? = null
    private var videoTrack: VideoTrack? = null
    private var videoSource: VideoSource? = null
    private var metadataChannel: DataChannel? = null
    private var metadataChannelReady = false
    private val executor = Executors.newSingleThreadExecutor()
    private val isDisposing = AtomicBoolean(false)
    private val appContext = context.applicationContext
    @Volatile private var currentFps: Int = options.fps
    
    var connectionListener: PeerConnectionListener? = null

    /**
     * Change the streaming resolution on-the-fly without renegotiation.
     * The capturer will scale the next frame to the new dimensions.
     */
    fun changeResolution(width: Int, height: Int) {
        when (videoCapturer) {
            is DJIV5VideoCapturer -> videoCapturer.changeResolution(width, height)
            is SharedVideoCapturerHandle -> videoCapturer.changeResolution(width, height)
            is MockMp4VideoCapturer -> videoCapturer.changeResolution(width, height)
            else -> return
        }
        Log.d(TAG, "[$clientId] Resolution changed to ${width}x${height}")
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
        Log.d(TAG, "[$clientId] Frame rate changed to $boundedFps fps")
    }

    interface PeerConnectionListener {
        fun onConnected(clientId: String)
        fun onDisconnected(clientId: String)
        fun onError(clientId: String, error: String)
    }

    init {
        executor.execute {
            createVideoTrack()
            initializePeerConnection()
            setupMetadataListener()
            // Create data channel AFTER peer connection is ready
            createMetadataDataChannel()
            startStreaming()
        }
    }

    private fun createVideoTrack() {
        Log.d(TAG, "Creating video track for client: $clientId")
        
        val factory = getFactory(appContext)
        videoSource = factory.createVideoSource(false)
        
        videoCapturer.initialize(
            null,  // SurfaceTextureHelper not needed for our custom capturer
            appContext,
            videoSource!!.capturerObserver
        )
        videoCapturer.startCapture(
            options.videoResolutionWidth,
            options.videoResolutionHeight,
            options.fps
        )
        
        videoTrack = factory.createVideoTrack(options.videoTrackId, videoSource)
        videoTrack?.setEnabled(true)
        
        Log.d(TAG, "Video track created: ${videoTrack?.id()}")
    }

    private fun initializePeerConnection() {
        Log.d(TAG, "Initializing peer connection for client: $clientId")
        
        // For local network, we don't need STUN/TURN servers
        // But we add Google's STUN as fallback for network discovery
        val iceServers = listOf(
            PeerConnection.IceServer.builder("stun:stun.l.google.com:19302")
                .createIceServer()
        )
        
        val rtcConfig = PeerConnection.RTCConfiguration(iceServers).apply {
            sdpSemantics = PeerConnection.SdpSemantics.UNIFIED_PLAN
            continualGatheringPolicy = PeerConnection.ContinualGatheringPolicy.GATHER_CONTINUALLY
        }

        // Prefer stable resolution over adaptive CPU downscale.
        disableCpuOveruseDetection(rtcConfig)
        
        peerConnection = getFactory(appContext).createPeerConnection(rtcConfig, createPeerConnectionObserver())
        
        Log.d(TAG, "Peer connection created")
    }

    private fun createMetadataDataChannel() {
        Log.d(TAG, "Creating metadata data channel for client: $clientId")
        
        val config = DataChannel.Init().apply {
            ordered = true       // Ensure metadata arrives in order
            negotiated = true    // Both sides create the channel with same id
            id = 0              // Use specific id for negotiated channels
            maxRetransmits = 0   // Don't retransmit old metadata (we want real-time)
        }
        
        try {
            metadataChannel = peerConnection?.createDataChannel(METADATA_CHANNEL_LABEL, config)
            
            metadataChannel?.registerObserver(object : DataChannel.Observer {
                override fun onBufferedAmountChange(previousAmount: Long) {}
                
                override fun onStateChange() {
                    val state = metadataChannel?.state()
                    Log.d(TAG, "[$clientId] Metadata channel state: $state")
                    if (state == DataChannel.State.OPEN) {
                        metadataChannelReady = true
                        Log.d(TAG, "[$clientId] Metadata channel OPEN and ready to send")
                    } else {
                        metadataChannelReady = false
                    }
                }
                
                override fun onMessage(buffer: DataChannel.Buffer) {
                    // We don't expect incoming messages on this channel
                }
            })
            
            Log.d(TAG, "Metadata data channel created: ${metadataChannel?.label()}")
        } catch (e: Exception) {
            Log.e(TAG, "Error creating metadata data channel: ${e.message}", e)
        }
    }
    
    private fun setupMetadataListener() {
        // Set up listener on the video capturer to receive frame metadata
        Log.d(TAG, "[$clientId] Setting up metadata listener on video capturer")
        
        val metadataCallback = object : DJIV5VideoCapturer.FrameMetadataListener {
            override fun onFrameMetadata(metadata: FrameMetadata) {
                Log.v(TAG, "[$clientId] Metadata listener received frame: ${metadata.frameNumber}")
                sendMetadata(metadata)
            }
        }
        when (videoCapturer) {
            is DJIV5VideoCapturer -> videoCapturer.metadataListener = metadataCallback
            is SharedVideoCapturerHandle -> videoCapturer.metadataListener = metadataCallback
            is MockMp4VideoCapturer -> videoCapturer.metadataListener = metadataCallback
            else -> {
                Log.w(TAG, "[$clientId] VideoCapturer type not supported for metadata")
                return
            }
        }
        Log.d(TAG, "[$clientId] Metadata listener attached to video capturer")
    }
    
    /**
     * Send frame metadata through the data channel.
     * Drops metadata when the channel is congested to avoid unbounded buffering.
     */
    private fun sendMetadata(metadata: FrameMetadata) {
        if (!metadataChannelReady || metadataChannel == null) {
            return  // Channel not ready yet
        }
        
        try {
            val channel = metadataChannel ?: return
            if (channel.bufferedAmount() > METADATA_BUFFER_THRESHOLD) {
                Log.w(TAG, "[$clientId] DataChannel congested (${channel.bufferedAmount()} bytes buffered), dropping metadata")
                return
            }
            val jsonString = metadata.toJsonString()
            val buffer = ByteBuffer.wrap(jsonString.toByteArray(StandardCharsets.UTF_8))
            channel.send(DataChannel.Buffer(buffer, false))  // false = text data
            Log.d(TAG, "[$clientId] Sent metadata for frame ${metadata.frameNumber}")
        } catch (e: Exception) {
            Log.e(TAG, "[$clientId] Error sending metadata: ${e.message}")
        }
    }

    private fun startStreaming() {
        Log.d(TAG, "Starting video streaming for client: $clientId")
        
        videoTrack?.let { track ->
            val streamIds = listOf(options.mediaStreamId)
            val sender = peerConnection?.addTrack(track, streamIds)
            if (sender != null) {
                configureVideoSenderForStability(sender)
            }
            Log.d(TAG, "Video track added to peer connection")
        }
    }

    private fun disableCpuOveruseDetection(rtcConfig: PeerConnection.RTCConfiguration) {
        runCatching {
            val field = rtcConfig.javaClass.getField("enableCpuOveruseDetection")
            field.isAccessible = true
            field.setBoolean(rtcConfig, false)
            Log.d(TAG, "[$clientId] Disabled RTC CPU overuse detection")
        }.onFailure {
            Log.d(TAG, "[$clientId] RTC CPU overuse flag unavailable on this WebRTC build")
        }
    }

    private fun configureVideoSenderForStability(sender: RtpSender) {
        runCatching {
            val params = sender.parameters ?: return
            val encodings = params.encodings ?: emptyList()
            val bitrateCap = options.senderBitrateBps()

            encodings.forEach { encoding ->
                runCatching {
                    encoding.maxBitrateBps = bitrateCap
                }
                runCatching {
                    encoding.maxFramerate = currentFps
                }
            }

            // Force adaptation strategy toward resolution reduction before FPS reduction.
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
            Log.d(
                TAG,
                "[$clientId] Sender params tuned: maxBitrate=${bitrateCap}bps, maxFps=$currentFps, prefer=MAINTAIN_FRAMERATE"
            )
        }.onFailure { e ->
            Log.w(TAG, "[$clientId] Unable to fully apply sender tuning: ${e.message}")
        }
    }

    private fun createPeerConnectionObserver(): PeerConnection.Observer {
        return object : PeerConnection.Observer {
            override fun onSignalingChange(state: PeerConnection.SignalingState) {
                Log.d(TAG, "[$clientId] Signaling state: $state")
            }

            override fun onIceConnectionChange(state: PeerConnection.IceConnectionState) {
                Log.d(TAG, "[$clientId] ICE connection state: $state")
                when (state) {
                    PeerConnection.IceConnectionState.CONNECTED -> {
                        connectionListener?.onConnected(clientId)
                    }
                    PeerConnection.IceConnectionState.DISCONNECTED,
                    PeerConnection.IceConnectionState.FAILED -> {
                        connectionListener?.onDisconnected(clientId)
                    }
                    PeerConnection.IceConnectionState.CLOSED -> {
                        connectionListener?.onDisconnected(clientId)
                    }
                    else -> {}
                }
            }

            override fun onIceConnectionReceivingChange(receiving: Boolean) {
                Log.d(TAG, "[$clientId] ICE connection receiving: $receiving")
            }

            override fun onIceGatheringChange(state: PeerConnection.IceGatheringState) {
                Log.d(TAG, "[$clientId] ICE gathering state: $state")
            }

            override fun onIceCandidate(candidate: IceCandidate) {
                Log.d(TAG, "[$clientId] Local ICE candidate: ${candidate.sdpMid}")
                sendIceCandidate(candidate)
            }

            override fun onIceCandidatesRemoved(candidates: Array<out IceCandidate>) {
                Log.d(TAG, "[$clientId] ICE candidates removed")
            }

            override fun onAddStream(stream: MediaStream) {
                Log.d(TAG, "[$clientId] Stream added: ${stream.id}")
            }

            override fun onRemoveStream(stream: MediaStream) {
                Log.d(TAG, "[$clientId] Stream removed: ${stream.id}")
            }

            override fun onDataChannel(dataChannel: DataChannel) {
                Log.d(TAG, "[$clientId] Data channel: ${dataChannel.label()}")
            }

            override fun onRenegotiationNeeded() {
                Log.d(TAG, "[$clientId] Renegotiation needed")
            }

            override fun onAddTrack(receiver: RtpReceiver, streams: Array<out MediaStream>) {
                Log.d(TAG, "[$clientId] Track added")
            }
        }
    }

    /**
     * Handle incoming WebRTC signaling message from the client
     */
    fun handleSignalingMessage(message: JSONObject) {
        executor.execute {
            try {
                when (message.getString("type")) {
                    "offer" -> handleOffer(message)
                    "answer" -> handleAnswer(message)
                    "candidate" -> handleIceCandidate(message)
                    else -> Log.w(TAG, "Unknown message type: ${message.optString("type")}")
                }
            } catch (e: JSONException) {
                Log.e(TAG, "Error handling message: ${e.message}", e)
            }
        }
    }

    private fun handleOffer(message: JSONObject) {
        Log.d(TAG, "[$clientId] Handling offer")
        
        val sdp = message.getString("sdp")
        val sessionDescription = SessionDescription(SessionDescription.Type.OFFER, sdp)
        
        peerConnection?.setRemoteDescription(SimpleSdpObserver(TAG), sessionDescription)
        
        // Create and send answer
        val constraints = MediaConstraints().apply {
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveVideo", "false"))
            mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveAudio", "false"))
            optional.add(MediaConstraints.KeyValuePair("googCpuOveruseDetection", "false"))
        }
        
        peerConnection?.createAnswer(object : SimpleSdpObserver(TAG) {
            override fun onCreateSuccess(sessionDescription: SessionDescription?) {
                sessionDescription?.let { sdp ->
                    val mungedSdp = SessionDescription(
                        sdp.type,
                        SdpUtils.mungeForH264(sdp.description)
                    )
                    peerConnection?.setLocalDescription(SimpleSdpObserver(TAG), mungedSdp)
                    sendAnswer(mungedSdp)
                }
            }
        }, constraints)
    }

    private fun handleAnswer(message: JSONObject) {
        Log.d(TAG, "[$clientId] Handling answer")
        
        val sdp = message.getString("sdp")
        val sessionDescription = SessionDescription(SessionDescription.Type.ANSWER, sdp)
        peerConnection?.setRemoteDescription(SimpleSdpObserver(TAG), sessionDescription)
    }

    private fun handleIceCandidate(message: JSONObject) {
        Log.d(TAG, "[$clientId] Handling ICE candidate")
        
        val candidate = IceCandidate(
            message.getString("id"),
            message.getInt("label"),
            message.getString("candidate")
        )
        peerConnection?.addIceCandidate(candidate)
    }

    /**
     * Create and send an offer to the client (when we initiate the call)
     */
    fun createOffer() {
        executor.execute {
            val constraints = MediaConstraints().apply {
                mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveVideo", "false"))
                mandatory.add(MediaConstraints.KeyValuePair("OfferToReceiveAudio", "false"))
                optional.add(MediaConstraints.KeyValuePair("googCpuOveruseDetection", "false"))
            }
            
            peerConnection?.createOffer(object : SimpleSdpObserver(TAG) {
                override fun onCreateSuccess(sessionDescription: SessionDescription?) {
                    sessionDescription?.let { sdp ->
                        val mungedSdp = SessionDescription(
                            sdp.type,
                            SdpUtils.mungeForH264(sdp.description)
                        )
                        peerConnection?.setLocalDescription(SimpleSdpObserver(TAG), mungedSdp)
                        sendOffer(mungedSdp)
                    }
                }
            }, constraints)
        }
    }

    private fun sendOffer(sdp: SessionDescription) {
        val message = JSONObject().apply {
            put("type", "offer")
            put("sdp", sdp.description)
        }
        messageCallback(clientId, message)
    }

    private fun sendAnswer(sdp: SessionDescription) {
        val message = JSONObject().apply {
            put("type", "answer")
            put("sdp", sdp.description)
        }
        messageCallback(clientId, message)
    }

    private fun sendIceCandidate(candidate: IceCandidate) {
        val message = JSONObject().apply {
            put("type", "candidate")
            put("label", candidate.sdpMLineIndex)
            put("id", candidate.sdpMid)
            put("candidate", candidate.sdp)
        }
        messageCallback(clientId, message)
    }

    /**
     * Clean up resources.
     * @param onComplete optional callback invoked after disposal finishes.
     */
    fun dispose(onComplete: (() -> Unit)? = null) {
        if (!isDisposing.compareAndSet(false, true)) {
            onComplete?.invoke()
            return
        }

        Log.d(TAG, "Disposing WebRTCClient for: $clientId")
        connectionListener = null
        
        executor.execute {
            when (videoCapturer) {
                is DJIV5VideoCapturer -> videoCapturer.metadataListener = null
                is SharedVideoCapturerHandle -> videoCapturer.metadataListener = null
                is MockMp4VideoCapturer -> videoCapturer.metadataListener = null
            }

            try {
                videoCapturer.stopCapture()
            } catch (e: Exception) {
                Log.e(TAG, "Error stopping capturer: ${e.message}")
            }
            
            // Close data channel
            runCatching { metadataChannel?.close() }
            metadataChannel = null
            metadataChannelReady = false
            
            runCatching { videoCapturer.dispose() }
            runCatching { videoTrack?.setEnabled(false) }
            videoTrack = null
            videoSource = null
            
            // Avoid aggressive native disposal here. The crash reports show libjingle
            // aborting on its signaling thread, which is commonly triggered by
            // re-entrant or thread-unsafe shutdown. Closing the connection is enough
            // to stop network activity without forcing native teardown mid-callback.
            runCatching { peerConnection?.close() }
            peerConnection = null
            
            onComplete?.invoke()
        }
        
        executor.shutdown()
    }
}
