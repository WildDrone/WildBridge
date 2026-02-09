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
    private val context: Context,
    private val cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    private val signalingPort: Int = 8081,
    private val droneName: String = "drone_1",
    private val options: WebRTCMediaOptions = WebRTCMediaOptions()
) {

    companion object {
        private const val TAG = "WebRTCStreamer"
    }

    private var signalingServer: WebRTCSignalingServer? = null
    private val activeConnections = ConcurrentHashMap<String, WebRTCClient>()
    private val mainHandler = Handler(Looper.getMainLooper())
    
    var listener: WebRTCStreamerListener? = null

    interface WebRTCStreamerListener {
        fun onServerStarted(ip: String, port: Int)
        fun onServerStopped()
        fun onServerError(error: String)
        fun onClientConnected(clientId: String, totalClients: Int)
        fun onClientDisconnected(clientId: String, totalClients: Int)
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
     * Stop the WebRTC streaming server and all connections
     */
    fun stop() {
        Log.d(TAG, "Stopping WebRTC streamer...")
        
        // Close all peer connections
        activeConnections.values.forEach { client ->
            client.dispose()
        }
        activeConnections.clear()
        
        // Stop signaling server
        signalingServer?.stopServer()
        signalingServer = null
        
        mainHandler.post {
            listener?.onServerStopped()
        }
        
        Log.d(TAG, "WebRTC streamer stopped")
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
        activeConnections.values.forEach { client ->
            client.changeResolution(width, height)
        }
    }

    /**
     * Get the connection URL for clients
     */
    fun getConnectionUrl(): String {
        val ip = getLocalIpAddress() ?: "Unknown"
        return "ws://$ip:$signalingPort"
    }

    private fun createPeerConnection(clientId: String) {
        Log.d(TAG, "Creating peer connection for: $clientId")
        
        // Create a new video capturer for this client
        val videoCapturer = DJIV5VideoCapturer(cameraIndex, droneName = droneName)
        
        val client = WebRTCClient(
            clientId = clientId,
            context = context,
            videoCapturer = videoCapturer,
            options = options,
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
                removePeerConnection(clientId)
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
