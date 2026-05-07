package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import org.json.JSONObject
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
    private val options: WebRTCMediaOptions = WebRTCMediaOptions()
) {

    companion object {
        private const val TAG = "WebRTCStreamer"
    }

    private val appContext = context.applicationContext

    private var signalingServer: WebRTCSignalingServer? = null
    private val activeConnections = ConcurrentHashMap<String, WebRTCClient>()
    private val mainHandler = Handler(Looper.getMainLooper())
    private var sharedFrameSource: SharedDJIFrameSource? = null
    private var whipPublisher: WhipPublisher? = null
    @Volatile private var currentOptions: WebRTCMediaOptions = options
    @Volatile private var currentWhipUrl: String? = null
    private var badMetricsWindows = 0
    private var recoveryCount = 0
    private var lastRecoveryAtMs = 0L
    
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

    /**
     * Start publishing video via WHIP to a mediamtx relay server.
     * This replaces the signaling server approach — the phone pushes
     * its stream once and mediamtx fans it out to all consumers.
     *
     * @param whipUrl Full WHIP endpoint URL, e.g. "http://192.168.x.y:8889/drone_1/whip"
     */
    fun startWhip(whipUrl: String) {
        Log.d(TAG, "Starting WHIP publisher to $whipUrl")

        if (whipPublisher != null) {
            Log.w(TAG, "WHIP publisher already running")
            return
        }

        TelemetryProvider.startListening()
    currentWhipUrl = whipUrl

        // Use a SharedVideoCapturerHandle backed by the shared DJI frame source.
        // This is the same mechanism used for per-client WebRTC but only one instance.
        val capturer = SharedVideoCapturerHandle("whip", getOrCreateSharedSource())

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
                    mainHandler.post {
                        this@WebRTCStreamer.listener?.onServerStarted(ip, 0)
                    }
                }
                override fun onDisconnected() {
                    Log.w(TAG, "WHIP connection lost")
                }
                override fun onError(error: String) {
                    Log.e(TAG, "WHIP error: $error")
                    mainHandler.post {
                        this@WebRTCStreamer.listener?.onServerError("WHIP: $error")
                    }
                }
            }
            start()
        }
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
        Log.d(TAG, "Changing resolution to ${width}x${height} for ${activeConnections.size} client(s)")
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
        changeResolution(options.videoResolutionWidth, options.videoResolutionHeight)
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

    private fun handleFrameSourceMetrics(metrics: WebRTCStreamMetrics) {
        val enriched = metrics.copy(
            recoveryCount = recoveryCount,
            status = if (whipPublisher != null || signalingServer != null) metrics.status else "idle"
        )
        maybeRecoverStreaming(enriched)
        mainHandler.post { listener?.onMetrics(enriched) }
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
        if (badMetricsWindows >= 3 && now - lastRecoveryAtMs > 15_000L) {
            lastRecoveryAtMs = now
            recoveryCount++
            badMetricsWindows = 0
            val reason = "low output fps ${metrics.outputFps} with input fps ${metrics.inputFps}"
            Log.w(TAG, "Recovering WebRTC pipeline: $reason")
            sharedFrameSource?.recoverCapture(reason)
            restartWhipPublisher(reason)
        }
    }

    private fun restartWhipPublisher(reason: String) {
        val whipUrl = currentWhipUrl ?: return
        val oldPublisher = whipPublisher ?: return
        Log.w(TAG, "Restarting WHIP publisher: $reason")
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
        val videoCapturer = SharedVideoCapturerHandle(clientId, getOrCreateSharedSource())
        
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
}
