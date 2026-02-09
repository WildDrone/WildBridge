package dji.sampleV5.aircraft.webrtc

import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sampleV5.aircraft.controller.DroneController
import dji.v5.et.create
import dji.v5.et.get

/**
 * Provides synchronized telemetry data from DJI SDK KeyManager.
 * 
 * This class captures drone telemetry at the exact moment a frame is received,
 * ensuring the metadata is synchronized with the video frame.
 */
object TelemetryProvider {
    
    // DJI Keys for telemetry - created once and reused
    private val location3DKey = FlightControllerKey.KeyAircraftLocation3D.create()
    private val altitudeKey = FlightControllerKey.KeyAltitude.create()
    private val attitudeKey = FlightControllerKey.KeyAircraftAttitude.create()
    private val velocityKey = FlightControllerKey.KeyAircraftVelocity.create()
    private val headingKey = FlightControllerKey.KeyCompassHeading.create()
    private val gimbalAttitudeKey = GimbalKey.KeyGimbalAttitude.create()
    private val satelliteCountKey = FlightControllerKey.KeyGPSSatelliteCount.create()
    private val batteryPercentKey = BatteryKey.KeyChargeRemainingInPercent.create()
    private val isFlyingKey = FlightControllerKey.KeyIsFlying.create()
    private val flightModeStringKey = FlightControllerKey.KeyFlightModeString.create()
    
    /**
     * Capture current telemetry state.
     * Call this at the moment each video frame is received for synchronization.
     * 
     * @param frameNumber The sequential frame number
     * @param timestampNs The frame timestamp in nanoseconds
     * @param frameWidth The video frame width
     * @param frameHeight The video frame height
     * @param droneName The name/identifier of this drone
     * @return FrameMetadata containing all telemetry synchronized with the frame
     */
    fun captureMetadata(
        frameNumber: Long,
        timestampNs: Long,
        frameWidth: Int,
        frameHeight: Int,
        droneName: String = "drone_1"
    ): FrameMetadata {
        // Capture all telemetry values at this moment
        val location = location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))
        val altitudeAGL = altitudeKey.get(0.0) ?: 0.0
        val attitude = attitudeKey.get(Attitude(0.0, 0.0, 0.0))
        val velocity = velocityKey.get(Velocity3D(0.0, 0.0, 0.0))
        val heading = headingKey.get(0.0) ?: 0.0
        val gimbalAttitude = gimbalAttitudeKey.get(Attitude(0.0, 0.0, 0.0))
        val satelliteCount = satelliteCountKey.get(0) ?: 0
        val batteryPercent = batteryPercentKey.get(0) ?: 0
        val isFlying = isFlyingKey.get(false) ?: false
        val flightMode = flightModeStringKey.get("") ?: "UNKNOWN"
        
        return FrameMetadata(
            frameNumber = frameNumber,
            timestampNs = timestampNs,
            captureTimeMs = System.currentTimeMillis(),
            droneName = droneName,
            frameWidth = frameWidth,
            frameHeight = frameHeight,
            
            // Position - location3D contains GPS altitude (ASL)
            latitude = location.latitude,
            longitude = location.longitude,
            altitudeASL = location.altitude,
            altitudeAGL = altitudeAGL,
            
            // Aircraft attitude
            aircraftPitch = attitude.pitch,
            aircraftRoll = attitude.roll,
            aircraftYaw = heading,  // Use compass heading for yaw
            
            // Gimbal attitude
            gimbalPitch = gimbalAttitude.pitch,
            gimbalRoll = gimbalAttitude.roll,
            gimbalYaw = gimbalAttitude.yaw,
            
            // Velocity (NED coordinate system)
            velocityX = velocity.x,
            velocityY = velocity.y,
            velocityZ = velocity.z,
            
            // Additional info
            satelliteCount = satelliteCount,
            batteryPercent = batteryPercent,
            isFlying = isFlying,
            flightMode = flightMode,
            isManualOverrideActive = DroneController.isManualOverrideActive
        )
    }
}
