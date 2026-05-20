package dji.sampleV5.aircraft.webrtc

import android.util.Log
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sampleV5.aircraft.controller.DroneController
import dji.v5.manager.KeyManager
import dji.v5.ux.detection.DetectedTarget
import dji.v5.et.create

/**
 * Provides synchronized telemetry data from DJI SDK KeyManager.
 * 
 * Values are cached via asynchronous KeyManager listeners so that
 * [captureMetadata] only reads pre-cached volatile fields instead of
 * issuing 11 blocking SDK calls per frame.
 */
object TelemetryProvider {

    private const val TAG = "TelemetryProvider"
    
    /** Current detected targets from AutoSensing – updated by the activity */
    @Volatile
    var currentDetectedTargets: List<DetectedTarget> = emptyList()
    
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

    // ---- Cached values updated asynchronously by KeyManager listeners ----
    @Volatile private var cachedLocation = LocationCoordinate3D(0.0, 0.0, 0.0)
    @Volatile private var cachedAltitude = 0.0
    @Volatile private var cachedAttitude = Attitude(0.0, 0.0, 0.0)
    @Volatile private var cachedVelocity = Velocity3D(0.0, 0.0, 0.0)
    @Volatile private var cachedHeading = 0.0
    @Volatile private var cachedGimbalAttitude = Attitude(0.0, 0.0, 0.0)
    @Volatile private var cachedSatelliteCount = 0
    @Volatile private var cachedBatteryPercent = 0
    @Volatile private var cachedIsFlying = false
    @Volatile private var cachedFlightMode = "UNKNOWN"
    @Volatile private var mockTelemetryEnabled = false
    @Volatile private var mockBaseLatitude = 55.6761
    @Volatile private var mockBaseLongitude = 12.5683
    @Volatile private var mockBaseAltitude = 24.0
    @Volatile private var mockStartTimeMs = System.currentTimeMillis()

    @Volatile private var listenersRegistered = false

    /**
     * Start listening for telemetry updates.  Safe to call multiple times;
     * listeners are only registered once.
     */
    fun startListening() {
        if (listenersRegistered) return
        listenersRegistered = true
        Log.d(TAG, "Registering async telemetry listeners")

        val km = KeyManager.getInstance()

        km.listen(location3DKey, this) { _, v ->
            v?.let { cachedLocation = it }
        }
        km.listen(altitudeKey, this) { _, v ->
            v?.let { cachedAltitude = it }
        }
        km.listen(attitudeKey, this) { _, v ->
            v?.let { cachedAttitude = it }
        }
        km.listen(velocityKey, this) { _, v ->
            v?.let { cachedVelocity = it }
        }
        km.listen(headingKey, this) { _, v ->
            v?.let { cachedHeading = it }
        }
        km.listen(gimbalAttitudeKey, this) { _, v ->
            v?.let { cachedGimbalAttitude = it }
        }
        km.listen(satelliteCountKey, this) { _, v ->
            v?.let { cachedSatelliteCount = it }
        }
        km.listen(batteryPercentKey, this) { _, v ->
            v?.let { cachedBatteryPercent = it }
        }
        km.listen(isFlyingKey, this) { _, v ->
            v?.let { cachedIsFlying = it }
        }
        km.listen(flightModeStringKey, this) { _, v ->
            v?.let { cachedFlightMode = it }
        }
    }

    /**
     * Stop all telemetry listeners.
     */
    fun stopListening() {
        if (!listenersRegistered) return
        listenersRegistered = false
        Log.d(TAG, "Cancelling async telemetry listeners")
        KeyManager.getInstance().cancelListen(this)
    }

    data class MockTelemetrySnapshot(
        val location: LocationCoordinate3D,
        val altitudeAGL: Double,
        val attitude: Attitude,
        val velocity: Velocity3D,
        val heading: Double,
        val gimbalAttitude: Attitude,
        val satelliteCount: Int,
        val batteryPercent: Int,
        val isFlying: Boolean,
        val flightMode: String
    )

    fun configureMockTelemetry(enabled: Boolean, baseLatitude: Double?, baseLongitude: Double?, baseAltitude: Double?) {
        if (enabled && !mockTelemetryEnabled) {
            mockStartTimeMs = System.currentTimeMillis()
        }
        mockTelemetryEnabled = enabled
        if (baseLatitude != null && baseLongitude != null && (baseLatitude != 0.0 || baseLongitude != 0.0)) {
            mockBaseLatitude = baseLatitude
            mockBaseLongitude = baseLongitude
        }
        if (baseAltitude != null && baseAltitude.isFinite()) {
            mockBaseAltitude = baseAltitude
        }
    }

    fun isMockTelemetryEnabled(): Boolean = mockTelemetryEnabled

    fun currentMockTelemetry(droneName: String = "drone_1"): MockTelemetrySnapshot {
        val elapsedSeconds = (System.currentTimeMillis() - mockStartTimeMs).coerceAtLeast(0L) / 1000.0
        val phase = ((droneName.hashCode().ushr(1) % 360) / 180.0) * Math.PI
        val angle = elapsedSeconds * 0.18 + phase
        val radiusMeters = 18.0
        val latOffset = (radiusMeters * kotlin.math.cos(angle)) / 111_320.0
        val lonScale = kotlin.math.cos(Math.toRadians(mockBaseLatitude)).coerceAtLeast(0.2)
        val lonOffset = (radiusMeters * kotlin.math.sin(angle)) / (111_320.0 * lonScale)
        val altitudeAGL = 22.0 + kotlin.math.sin(angle * 0.7) * 5.0
        val velocityNorth = -radiusMeters * 0.18 * kotlin.math.sin(angle)
        val velocityEast = radiusMeters * 0.18 * kotlin.math.cos(angle)
        val heading = (Math.toDegrees(angle) + 360.0) % 360.0
        val batteryPercent = (92 - (elapsedSeconds / 90.0).toInt()).coerceIn(55, 100)
        val gimbalPitch = -12.0 + kotlin.math.sin(angle * 0.5) * 6.0

        return MockTelemetrySnapshot(
            location = LocationCoordinate3D(
                mockBaseLatitude + latOffset,
                mockBaseLongitude + lonOffset,
                mockBaseAltitude + altitudeAGL
            ),
            altitudeAGL = altitudeAGL,
            attitude = Attitude(
                kotlin.math.sin(angle * 0.6) * 4.0,
                kotlin.math.cos(angle * 0.5) * 3.0,
                heading
            ),
            velocity = Velocity3D(velocityNorth, velocityEast, kotlin.math.cos(angle * 0.7) * -0.3),
            heading = heading,
            gimbalAttitude = Attitude(gimbalPitch, 0.0, heading),
            satelliteCount = 19,
            batteryPercent = batteryPercent,
            isFlying = true,
            flightMode = FlightMode.GPS_NORMAL.name
        )
    }
    
    /**
     * Capture current telemetry state.
     * Reads only pre-cached volatile fields — no SDK calls.
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
        if (mockTelemetryEnabled) {
            val mock = currentMockTelemetry(droneName)
            return FrameMetadata(
                frameNumber = frameNumber,
                timestampNs = timestampNs,
                captureTimeMs = System.currentTimeMillis(),
                droneName = droneName,
                frameWidth = frameWidth,
                frameHeight = frameHeight,
                latitude = mock.location.latitude,
                longitude = mock.location.longitude,
                altitudeASL = mock.location.altitude,
                altitudeAGL = mock.altitudeAGL,
                aircraftPitch = mock.attitude.pitch,
                aircraftRoll = mock.attitude.roll,
                aircraftYaw = mock.heading,
                gimbalPitch = mock.gimbalAttitude.pitch,
                gimbalRoll = mock.gimbalAttitude.roll,
                gimbalYaw = mock.gimbalAttitude.yaw,
                velocityX = mock.velocity.x,
                velocityY = mock.velocity.y,
                velocityZ = mock.velocity.z,
                satelliteCount = mock.satelliteCount,
                batteryPercent = mock.batteryPercent,
                isFlying = mock.isFlying,
                flightMode = mock.flightMode,
                isManualOverrideActive = false,
                detectedTargets = currentDetectedTargets
            )
        }

        val location = cachedLocation
        val attitude = cachedAttitude
        val velocity = cachedVelocity
        val gimbalAttitude = cachedGimbalAttitude

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
            altitudeAGL = cachedAltitude,
            
            // Aircraft attitude
            aircraftPitch = attitude.pitch,
            aircraftRoll = attitude.roll,
            aircraftYaw = cachedHeading,
            
            // Gimbal attitude
            gimbalPitch = gimbalAttitude.pitch,
            gimbalRoll = gimbalAttitude.roll,
            gimbalYaw = gimbalAttitude.yaw,
            
            // Velocity (NED coordinate system)
            velocityX = velocity.x,
            velocityY = velocity.y,
            velocityZ = velocity.z,
            
            // Additional info
            satelliteCount = cachedSatelliteCount,
            batteryPercent = cachedBatteryPercent,
            isFlying = cachedIsFlying,
            flightMode = cachedFlightMode,
            isManualOverrideActive = DroneController.isManualOverrideActive,
            detectedTargets = currentDetectedTargets
        )
    }
}
