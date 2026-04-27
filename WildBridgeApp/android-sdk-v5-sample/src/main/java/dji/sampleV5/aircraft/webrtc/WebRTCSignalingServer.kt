package dji.sampleV5.aircraft.webrtc

import android.util.Log
import org.java_websocket.WebSocket
import org.java_websocket.handshake.ClientHandshake
import org.java_websocket.server.WebSocketServer
import org.json.JSONObject
import java.net.InetSocketAddress
import java.util.concurrent.ConcurrentHashMap

/**
 * WebSocket-based signaling server for WebRTC connections.
 * Runs on the local network to allow WebRTC clients to connect and exchange SDP/ICE candidates.
 */
class WebRTCSignalingServer(
    port: Int,
    private val droneName: String = "drone_1",
    private val listener: SignalingServerListener
) : WebSocketServer(InetSocketAddress(port)) {

    init {
        isReuseAddr = true
    }

    companion object {
        private const val TAG = "WebRTCSignalingServer"
    }

    // Map of client ID to WebSocket connection
    private val clients = ConcurrentHashMap<String, WebSocket>()
    
    // The drone (server) connection - there's only one
    private var droneConnection: WebSocket? = null

    interface SignalingServerListener {
        fun onServerStarted(port: Int)
        fun onServerError(error: String)
        fun onClientConnected(clientId: String)
        fun onClientDisconnected(clientId: String)
        fun onWebRTCMessage(clientId: String, message: JSONObject)
    }

    override fun onOpen(conn: WebSocket, handshake: ClientHandshake) {
        val clientId = generateClientId(conn)
        clients[clientId] = conn
        Log.d(TAG, "Client connected: $clientId from ${conn.remoteSocketAddress}")
        
        // Send the client their ID
        val welcomeMsg = JSONObject().apply {
            put("type", "welcome")
            put("clientId", clientId)
            put("droneName", droneName)
        }
        conn.send(welcomeMsg.toString())
        
        listener.onClientConnected(clientId)
    }

    override fun onClose(conn: WebSocket, code: Int, reason: String, remote: Boolean) {
        val clientId = getClientId(conn)
        if (clientId != null) {
            clients.remove(clientId)
            Log.d(TAG, "Client disconnected: $clientId, reason: $reason")
            listener.onClientDisconnected(clientId)
        }
    }

    override fun onMessage(conn: WebSocket, message: String) {
        try {
            val json = JSONObject(message)
            val clientId = getClientId(conn) ?: return
            
            Log.d(TAG, "Received message from $clientId: ${json.optString("type")}")
            
            when (json.optString("type")) {
                "register" -> {
                    // Client is registering as viewer
                    Log.d(TAG, "Client $clientId registered as viewer")
                }
                "offer", "answer", "candidate" -> {
                    // WebRTC signaling messages - forward to listener
                    listener.onWebRTCMessage(clientId, json)
                }
                else -> {
                    Log.w(TAG, "Unknown message type: ${json.optString("type")}")
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error parsing message: ${e.message}")
        }
    }

    override fun onError(conn: WebSocket?, ex: Exception) {
        Log.e(TAG, "WebSocket error: ${ex.message}", ex)
        listener.onServerError(ex.message ?: "Unknown error")
    }

    override fun onStart() {
        Log.d(TAG, "Signaling server started on port $port")
        listener.onServerStarted(port)
    }

    /**
     * Send a message to a specific client
     */
    fun sendToClient(clientId: String, message: JSONObject) {
        clients[clientId]?.let { conn ->
            if (conn.isOpen) {
                conn.send(message.toString())
                Log.d(TAG, "Sent message to $clientId: ${message.optString("type")}")
            }
        }
    }

    /**
     * Broadcast a message to all connected clients
     */
    fun broadcastMessage(message: JSONObject) {
        val msgStr = message.toString()
        clients.values.forEach { conn ->
            if (conn.isOpen) {
                conn.send(msgStr)
            }
        }
    }

    /**
     * Get all connected client IDs
     */
    fun getConnectedClients(): Set<String> = clients.keys.toSet()

    /**
     * Get the number of connected clients
     */
    fun getClientCount(): Int = clients.size

    private fun generateClientId(conn: WebSocket): String {
        return "client_${conn.remoteSocketAddress.address.hostAddress}_${System.currentTimeMillis()}"
    }

    private fun getClientId(conn: WebSocket): String? {
        return clients.entries.find { it.value == conn }?.key
    }

    fun stopServer() {
        try {
            stop(1000)
        } catch (e: Exception) {
            Log.e(TAG, "Error stopping server: ${e.message}")
        }
    }
}
