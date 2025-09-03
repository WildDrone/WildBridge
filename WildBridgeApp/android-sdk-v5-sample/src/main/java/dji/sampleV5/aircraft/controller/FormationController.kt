package dji.sampleV5.aircraft.controller

import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sampleV5.aircraft.util.ToastUtils
import dji.v5.et.create
import dji.v5.et.get
import org.java_websocket.WebSocket
import org.java_websocket.server.WebSocketServer
import org.java_websocket.client.WebSocketClient
import org.java_websocket.handshake.ClientHandshake
import org.java_websocket.handshake.ServerHandshake
import org.json.JSONObject
import java.net.InetSocketAddress
import java.net.URI
import kotlin.math.*

data class FormationConfig(
    val distanceBehind: Double = 1.0, // meters behind leader
    val altitudeOffset: Double = 2.0, // meters above leader
    val maxFormationDistance: Double = 10.0, // break formation if too far
    val formationType: FormationType = FormationType.BEHIND
)

enum class FormationType {
    BEHIND, BESIDE, DIAGONAL
}

enum class DroneRole {
    LEADER, FOLLOWER, NONE
}

data class DroneState(
    val position: LocationCoordinate3D,
    val heading: Double,
    val battery: Int,
    val isConnected: Boolean,
    val timestamp: Long = System.currentTimeMillis()
)

data class FormationCommand(
    val type: String,
    val data: Map<String, Any>
)

object FormationController {

    private const val TAG = "FormationController"
    private const val WEBSOCKET_PORT = 8765
    private const val UPDATE_INTERVAL = 100L // ms
    private const val CONNECTION_TIMEOUT = 5000L // ms

    // DJI SDK Keys for telemetry
    private val location3DKey = FlightControllerKey.KeyAircraftLocation3D.create()
    private val compassHeadKey = FlightControllerKey.KeyCompassHeading.create()
    private val batteryPercentKey = BatteryKey.KeyChargeRemainingInPercent.create()

    var role: DroneRole = DroneRole.NONE
        private set
    var isFormationActive = false
        private set
    var config = FormationConfig()
        private set

    private var webSocketServer: FormationWebSocketServer? = null
    private var webSocketClient: FormationWebSocketClient? = null
    private var leaderState: DroneState? = null
    private var followerState: DroneState? = null

    private val mainHandler = Handler(Looper.getMainLooper())
    private var formationUpdateRunnable: Runnable? = null

    // Safety flags
    private var emergencyStopRequested = false
    private var collisionAvoidanceActive = false

    fun initAsLeader() {
        role = DroneRole.LEADER
        startWebSocketServer()
        ToastUtils.showToast("Initialized as Formation Leader")
    }

    fun initAsFollower(leaderIpAddress: String) {
        role = DroneRole.FOLLOWER
        connectToLeader(leaderIpAddress)
        ToastUtils.showToast("Connecting to Leader at $leaderIpAddress")
    }

    fun startFormation() {
        if (role == DroneRole.NONE) {
            ToastUtils.showToast("Please select Leader or Follower role first")
            return
        }

        if (role == DroneRole.FOLLOWER && leaderState == null) {
            ToastUtils.showToast("Not connected to leader")
            return
        }

        isFormationActive = true
        emergencyStopRequested = false
        startFormationControl()

        // Notify other drone
        if (role == DroneRole.LEADER) {
            broadcastCommand("formation_start", mapOf("config" to configToMap()))
        } else {
            sendToLeader("formation_ready", mapOf("follower_ready" to true))
        }

        ToastUtils.showToast("Formation Started")
    }

    fun stopFormation() {
        isFormationActive = false
        stopFormationControl()

        // Notify other drone
        if (role == DroneRole.LEADER) {
            broadcastCommand("formation_stop", mapOf("reason" to "manual_stop"))
        } else {
            sendToLeader("formation_stop", mapOf("reason" to "manual_stop"))
        }

        // Stop drone movement
        DroneController.setStick(0F, 0F, 0F, 0F)
        ToastUtils.showToast("Formation Stopped")
    }

    fun emergencyStop() {
        emergencyStopRequested = true
        isFormationActive = false
        stopFormationControl()

        // Stop both drones immediately
        DroneController.setStick(0F, 0F, 0F, 0F)

        // Notify other drone
        if (role == DroneRole.LEADER) {
            broadcastCommand("emergency_stop", mapOf("reason" to "emergency"))
        } else {
            sendToLeader("emergency_stop", mapOf("reason" to "emergency"))
        }

        ToastUtils.showToast("EMERGENCY STOP ACTIVATED")
    }

    private fun startWebSocketServer() {
        try {
            webSocketServer = FormationWebSocketServer(InetSocketAddress(WEBSOCKET_PORT))
            webSocketServer?.start()
            Log.i(TAG, "WebSocket server started on port $WEBSOCKET_PORT")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start WebSocket server", e)
            ToastUtils.showToast("Failed to start server: ${e.message}")
        }
    }

    private fun connectToLeader(ipAddress: String) {
        try {
            val uri = URI("ws://$ipAddress:$WEBSOCKET_PORT")
            webSocketClient = FormationWebSocketClient(uri)
            webSocketClient?.connect()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to connect to leader", e)
            ToastUtils.showToast("Failed to connect: ${e.message}")
        }
    }

    private fun startFormationControl() {
        formationUpdateRunnable = object : Runnable {
            override fun run() {
                if (isFormationActive && !emergencyStopRequested) {
                    updateFormation()
                    mainHandler.postDelayed(this, UPDATE_INTERVAL)
                }
            }
        }
        mainHandler.post(formationUpdateRunnable!!)
    }

    private fun stopFormationControl() {
        formationUpdateRunnable?.let { mainHandler.removeCallbacks(it) }
        formationUpdateRunnable = null
    }

    private fun updateFormation() {
        when (role) {
            DroneRole.LEADER -> updateAsLeader()
            DroneRole.FOLLOWER -> updateAsFollower()
            DroneRole.NONE -> return
        }
    }

    private fun updateAsLeader() {
        val currentState = getCurrentDroneState()

        // Broadcast leader state to followers
        broadcastCommand("leader_state", mapOf(
            "position" to positionToMap(currentState.position),
            "heading" to currentState.heading,
            "battery" to currentState.battery,
            "timestamp" to currentState.timestamp
        ))

        // Check if follower is too far away
        followerState?.let { follower ->
            val distance = DroneController.calculateDistance(
                currentState.position.latitude, currentState.position.longitude,
                follower.position.latitude, follower.position.longitude
            )

            if (distance > config.maxFormationDistance) {
                ToastUtils.showToast("Follower too far away - breaking formation")
                stopFormation()
            }
        }
    }

    private fun updateAsFollower() {
        leaderState?.let { leader ->
            val currentState = getCurrentDroneState()

            // Check connection timeout
            if (System.currentTimeMillis() - leader.timestamp > CONNECTION_TIMEOUT) {
                ToastUtils.showToast("Lost connection to leader")
                stopFormation()
                return
            }

            // Calculate target position based on formation config
            val targetPosition = calculateFollowerTargetPosition(leader, config)

            // Check safety constraints
            val distance = DroneController.calculateDistance(
                currentState.position.latitude, currentState.position.longitude,
                leader.position.latitude, leader.position.longitude
            )

            if (distance > config.maxFormationDistance) {
                ToastUtils.showToast("Too far from leader - breaking formation")
                stopFormation()
                return
            }

            // Collision avoidance check
            if (checkCollisionRisk(currentState, leader)) {
                collisionAvoidanceActive = true
                executeCollisionAvoidance(currentState, leader)
                return
            } else {
                collisionAvoidanceActive = false
            }

            // Navigate to target position
            DroneController.navigateToWaypointWithPID(
                targetPosition.latitude,
                targetPosition.longitude,
                targetPosition.altitude,
                leader.heading
            )

            // Send follower state to leader
            sendToLeader("follower_state", mapOf(
                "position" to positionToMap(currentState.position),
                "heading" to currentState.heading,
                "battery" to currentState.battery,
                "collision_avoidance" to collisionAvoidanceActive
            ))
        }
    }

    private fun calculateFollowerTargetPosition(leader: DroneState, config: FormationConfig): LocationCoordinate3D {
        val leaderPos = leader.position
        val leaderHeading = leader.heading

        return when (config.formationType) {
            FormationType.BEHIND -> {
                // Position behind the leader
                val behindLat = leaderPos.latitude - (config.distanceBehind * cos(Math.toRadians(leaderHeading)) / 111320.0)
                val behindLon = leaderPos.longitude - (config.distanceBehind * sin(Math.toRadians(leaderHeading)) / (111320.0 * cos(Math.toRadians(leaderPos.latitude))))
                LocationCoordinate3D(behindLat, behindLon, leaderPos.altitude + config.altitudeOffset)
            }
            FormationType.BESIDE -> {
                // Position to the right side of the leader
                val sideHeading = leaderHeading + 90
                val sideLat = leaderPos.latitude + (config.distanceBehind * cos(Math.toRadians(sideHeading)) / 111320.0)
                val sideLon = leaderPos.longitude + (config.distanceBehind * sin(Math.toRadians(sideHeading)) / (111320.0 * cos(Math.toRadians(leaderPos.latitude))))
                LocationCoordinate3D(sideLat, sideLon, leaderPos.altitude + config.altitudeOffset)
            }
            FormationType.DIAGONAL -> {
                // Position diagonally behind and to the right
                val diagHeading = leaderHeading + 45
                val diagLat = leaderPos.latitude - (config.distanceBehind * cos(Math.toRadians(diagHeading)) / 111320.0)
                val diagLon = leaderPos.longitude - (config.distanceBehind * sin(Math.toRadians(diagHeading)) / (111320.0 * cos(Math.toRadians(leaderPos.latitude))))
                LocationCoordinate3D(diagLat, diagLon, leaderPos.altitude + config.altitudeOffset)
            }
        }
    }

    private fun checkCollisionRisk(follower: DroneState, leader: DroneState): Boolean {
        val distance = DroneController.calculateDistance(
            follower.position.latitude, follower.position.longitude,
            leader.position.latitude, leader.position.longitude
        )
        val altitudeDiff = abs(follower.position.altitude - leader.position.altitude)

        // Risk if too close horizontally and not enough vertical separation
        return distance < 0.5 && altitudeDiff < 1.0
    }

    private fun executeCollisionAvoidance(follower: DroneState, leader: DroneState) {
        // Simple collision avoidance: move up and away
        val avoidanceAltitude = leader.position.altitude + 3.0
        val awayHeading = DroneController.calculateBearing(
            leader.position.latitude, leader.position.longitude,
            follower.position.latitude, follower.position.longitude
        ).toDouble()

        val avoidanceLat = follower.position.latitude + (2.0 * cos(Math.toRadians(awayHeading)) / 111320.0)
        val avoidanceLon = follower.position.longitude + (2.0 * sin(Math.toRadians(awayHeading)) / (111320.0 * cos(Math.toRadians(follower.position.latitude))))

        DroneController.navigateToWaypointWithPID(avoidanceLat, avoidanceLon, avoidanceAltitude, awayHeading)
        ToastUtils.showToast("Collision avoidance active")
    }

    private fun getCurrentDroneState(): DroneState {
        return try {
            val position = location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))
            val heading = compassHeadKey.get(0.0)
            val battery = batteryPercentKey.get(0)

            DroneState(
                position = position,
                heading = heading,
                battery = battery,
                isConnected = true,
                timestamp = System.currentTimeMillis()
            )
        } catch (e: Exception) {
            Log.e(TAG, "Error getting drone state", e)
            DroneState(
                position = LocationCoordinate3D(0.0, 0.0, 0.0),
                heading = 0.0,
                battery = 0,
                isConnected = false
            )
        }
    }

    private fun broadcastCommand(type: String, data: Map<String, Any>) {
        webSocketServer?.broadcast(createCommandJson(type, data))
    }

    private fun sendToLeader(type: String, data: Map<String, Any>) {
        webSocketClient?.send(createCommandJson(type, data))
    }

    private fun createCommandJson(type: String, data: Map<String, Any>): String {
        val json = JSONObject()
        json.put("type", type)
        json.put("data", JSONObject(data))
        json.put("timestamp", System.currentTimeMillis())
        return json.toString()
    }

    private fun configToMap(): Map<String, Any> {
        return mapOf(
            "distanceBehind" to config.distanceBehind,
            "altitudeOffset" to config.altitudeOffset,
            "maxFormationDistance" to config.maxFormationDistance,
            "formationType" to config.formationType.name
        )
    }

    private fun positionToMap(position: LocationCoordinate3D): Map<String, Double> {
        return mapOf(
            "latitude" to position.latitude,
            "longitude" to position.longitude,
            "altitude" to position.altitude
        )
    }

    // WebSocket Server for Leader
    private class FormationWebSocketServer(address: InetSocketAddress) : WebSocketServer(address) {

        override fun onOpen(conn: WebSocket?, handshake: ClientHandshake?) {
            Log.i(TAG, "Follower connected: ${conn?.remoteSocketAddress}")
            Handler(Looper.getMainLooper()).post {
                ToastUtils.showToast("Follower connected")
            }
        }

        override fun onClose(conn: WebSocket?, code: Int, reason: String?, remote: Boolean) {
            Log.i(TAG, "Follower disconnected: $reason")
            Handler(Looper.getMainLooper()).post {
                ToastUtils.showToast("Follower disconnected")
                if (isFormationActive) {
                    stopFormation()
                }
            }
        }

        override fun onMessage(conn: WebSocket?, message: String?) {
            message?.let { handleIncomingMessage(it) }
        }

        override fun onError(conn: WebSocket?, ex: Exception?) {
            Log.e(TAG, "WebSocket server error", ex)
        }

        override fun onStart() {
            Log.i(TAG, "WebSocket server started successfully")
        }
    }

    // WebSocket Client for Follower
    private class FormationWebSocketClient(uri: URI) : WebSocketClient(uri) {

        override fun onOpen(handshake: ServerHandshake?) {
            Log.i(TAG, "Connected to leader")
            Handler(Looper.getMainLooper()).post {
                ToastUtils.showToast("Connected to leader")
            }
        }

        override fun onMessage(message: String?) {
            message?.let { handleIncomingMessage(it) }
        }

        override fun onClose(code: Int, reason: String?, remote: Boolean) {
            Log.i(TAG, "Disconnected from leader: $reason")
            Handler(Looper.getMainLooper()).post {
                ToastUtils.showToast("Disconnected from leader")
                if (isFormationActive) {
                    stopFormation()
                }
            }
        }

        override fun onError(ex: Exception?) {
            Log.e(TAG, "WebSocket client error", ex)
            Handler(Looper.getMainLooper()).post {
                ToastUtils.showToast("Connection error: ${ex?.message}")
            }
        }
    }

    private fun handleIncomingMessage(message: String) {
        try {
            val json = JSONObject(message)
            val type = json.getString("type")
            val data = json.getJSONObject("data")

            when (type) {
                "leader_state" -> {
                    if (role == DroneRole.FOLLOWER) {
                        val position = data.getJSONObject("position")
                        leaderState = DroneState(
                            position = LocationCoordinate3D(
                                position.getDouble("latitude"),
                                position.getDouble("longitude"),
                                position.getDouble("altitude")
                            ),
                            heading = data.getDouble("heading"),
                            battery = data.getInt("battery"),
                            isConnected = true,
                            timestamp = data.getLong("timestamp")
                        )
                    }
                }
                "follower_state" -> {
                    if (role == DroneRole.LEADER) {
                        val position = data.getJSONObject("position")
                        followerState = DroneState(
                            position = LocationCoordinate3D(
                                position.getDouble("latitude"),
                                position.getDouble("longitude"),
                                position.getDouble("altitude")
                            ),
                            heading = data.getDouble("heading"),
                            battery = data.getInt("battery"),
                            isConnected = true
                        )
                    }
                }
                "formation_start" -> {
                    if (role == DroneRole.FOLLOWER && !isFormationActive) {
                        startFormation()
                    }
                }
                "formation_stop", "emergency_stop" -> {
                    if (isFormationActive) {
                        stopFormation()
                    }
                    if (type == "emergency_stop") {
                        emergencyStopRequested = true
                        DroneController.setStick(0F, 0F, 0F, 0F)
                    }
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error parsing message: $message", e)
        }
    }

    fun getConnectionStatus(): String {
        return when (role) {
            DroneRole.LEADER -> if (webSocketServer?.connections?.isNotEmpty() == true) "Follower Connected" else "Waiting for Follower"
            DroneRole.FOLLOWER -> if (webSocketClient?.isOpen == true) "Connected to Leader" else "Disconnected"
            DroneRole.NONE -> "No Role Selected"
        }
    }

    fun getFormationStatus(): String {
        return when {
            emergencyStopRequested -> "EMERGENCY STOP"
            collisionAvoidanceActive -> "COLLISION AVOIDANCE"
            isFormationActive -> "Formation Active"
            else -> "Formation Inactive"
        }
    }

    fun getOtherDroneState(): DroneState? {
        return when (role) {
            DroneRole.LEADER -> followerState
            DroneRole.FOLLOWER -> leaderState
            DroneRole.NONE -> null
        }
    }

    fun cleanup() {
        stopFormation()
        webSocketServer?.stop()
        webSocketClient?.close()
        role = DroneRole.NONE
    }
}
