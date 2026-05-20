package dji.sampleV5.aircraft.webrtc

import dji.v5.ux.detection.DetectedTarget
import org.json.JSONArray
import org.json.JSONObject

/**
 * Frame metadata containing telemetry synchronized with video frames.
 * This data is captured at the exact moment each frame is received from the drone camera.
 */
data class FrameMetadata(
    // Frame identification
    val frameNumber: Long,
    val timestampNs: Long,
    val captureTimeMs: Long,
    
    // Drone identification
    val droneName: String,
    
    // Frame properties
    val frameWidth: Int,
    val frameHeight: Int,
    
    // Aircraft position
    val latitude: Double,
    val longitude: Double,
    val altitudeASL: Double,      // Above sea level (GPS altitude)
    val altitudeAGL: Double,      // Above ground level (relative to takeoff)
    
    // Aircraft attitude (degrees)
    val aircraftPitch: Double,
    val aircraftRoll: Double,
    val aircraftYaw: Double,      // Heading / compass direction
    
    // Gimbal attitude (degrees)
    val gimbalPitch: Double,
    val gimbalRoll: Double,
    val gimbalYaw: Double,
    
    // Velocity (m/s)
    val velocityX: Double,        // North
    val velocityY: Double,        // East  
    val velocityZ: Double,        // Down (positive = descending)
    
    // Additional info
    val satelliteCount: Int,
    val batteryPercent: Int,
    val isFlying: Boolean,
    val flightMode: String,          // DJI flight mode string (e.g. "GPS", "ATTI", "SPORT", "TRIPOD")
    val isManualOverrideActive: Boolean = false,  // True when pilot has taken manual RC control
    val detectedTargets: List<DetectedTarget> = emptyList()  // AI-detected targets from AutoSensing
) {
    /**
     * Convert to JSON for transmission via WebRTC data channel
     */
    fun toJson(): JSONObject {
        return JSONObject().apply {
            // Frame info
            put("frameNumber", frameNumber)
            put("timestampNs", timestampNs)
            put("captureTimeMs", captureTimeMs)
            put("droneName", droneName)
            put("frameWidth", frameWidth)
            put("frameHeight", frameHeight)
            
            // Position
            put("latitude", latitude)
            put("longitude", longitude)
            put("altitudeASL", altitudeASL)
            put("altitudeAGL", altitudeAGL)
            
            // Aircraft attitude
            put("aircraftPitch", aircraftPitch)
            put("aircraftRoll", aircraftRoll)
            put("aircraftYaw", aircraftYaw)
            
            // Gimbal attitude
            put("gimbalPitch", gimbalPitch)
            put("gimbalRoll", gimbalRoll)
            put("gimbalYaw", gimbalYaw)
            
            // Velocity
            put("velocityX", velocityX)
            put("velocityY", velocityY)
            put("velocityZ", velocityZ)
            
            // Additional
            put("satelliteCount", satelliteCount)
            put("batteryPercent", batteryPercent)
            put("isFlying", isFlying)
            put("flightMode", flightMode)
            put("isManualOverrideActive", isManualOverrideActive)
            put("detectedTargets", JSONArray(detectedTargets.map { it.toJson() }))
        }
    }
    
    /**
     * Convert to compact JSON string for transmission
     */
    fun toJsonString(): String = toJson().toString()
    
    companion object {
        /**
         * Parse from JSON string received via data channel
         */
        fun fromJson(json: String): FrameMetadata {
            val obj = JSONObject(json)
            return FrameMetadata(
                frameNumber = obj.optLong("frameNumber", 0),
                timestampNs = obj.optLong("timestampNs", 0),
                captureTimeMs = obj.optLong("captureTimeMs", 0),
                droneName = obj.optString("droneName", "unknown"),
                frameWidth = obj.optInt("frameWidth", 0),
                frameHeight = obj.optInt("frameHeight", 0),
                latitude = obj.optDouble("latitude", 0.0),
                longitude = obj.optDouble("longitude", 0.0),
                altitudeASL = obj.optDouble("altitudeASL", 0.0),
                altitudeAGL = obj.optDouble("altitudeAGL", 0.0),
                aircraftPitch = obj.optDouble("aircraftPitch", 0.0),
                aircraftRoll = obj.optDouble("aircraftRoll", 0.0),
                aircraftYaw = obj.optDouble("aircraftYaw", 0.0),
                gimbalPitch = obj.optDouble("gimbalPitch", 0.0),
                gimbalRoll = obj.optDouble("gimbalRoll", 0.0),
                gimbalYaw = obj.optDouble("gimbalYaw", 0.0),
                velocityX = obj.optDouble("velocityX", 0.0),
                velocityY = obj.optDouble("velocityY", 0.0),
                velocityZ = obj.optDouble("velocityZ", 0.0),
                satelliteCount = obj.optInt("satelliteCount", 0),
                batteryPercent = obj.optInt("batteryPercent", 0),
                isFlying = obj.optBoolean("isFlying", false),
                flightMode = obj.optString("flightMode", "UNKNOWN"),
                isManualOverrideActive = obj.optBoolean("isManualOverrideActive", false),
                detectedTargets = DetectedTarget.fromJsonArray(obj.optJSONArray("detectedTargets") ?: JSONArray())
            )
        }
    }
}
