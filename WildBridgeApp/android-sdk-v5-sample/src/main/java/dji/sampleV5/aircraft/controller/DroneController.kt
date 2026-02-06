package dji.sampleV5.aircraft.controller

import android.os.Handler
import android.os.Looper
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.virtualstick.Stick
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.*
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sampleV5.moduleaircraft.controller.PID
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.v5.et.create
import dji.v5.et.get
import kotlin.math.*
import com.dji.wpmzsdk.common.data.Template
import com.dji.wpmzsdk.manager.WPMZManager
import dji.sampleV5.aircraft.utils.wpml.WaypointInfoModel
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.utils.common.ContextUtil
import dji.sdk.wpmz.value.mission.*
import dji.sdk.wpmz.value.mission.WaylineActionInfo
import dji.sdk.wpmz.value.mission.WaylineActionType
import dji.sdk.wpmz.value.mission.ActionGimbalRotateParam
import dji.sdk.wpmz.value.mission.WaylineGimbalActuatorRotateMode
import dji.sdk.wpmz.value.mission.WaylineActionGroup
import dji.sdk.wpmz.value.mission.WaylineActionTrigger
import dji.sdk.wpmz.value.mission.WaylineActionTriggerType
import dji.sdk.wpmz.value.mission.WaylineActionNodeList
import dji.sdk.wpmz.value.mission.WaylineActionTreeNode
import dji.sdk.wpmz.value.mission.WaylineActionsRelationType
import dji.v5.et.set
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.zip.ZipEntry
import java.util.zip.ZipFile


object DroneController {

    private var basicAircraftControlVM: BasicAircraftControlVM? = null
    var virtualStickVM: VirtualStickVM? = null

    fun init(basicVM: BasicAircraftControlVM, stickVM: VirtualStickVM ) {
        basicAircraftControlVM = basicVM
        virtualStickVM = stickVM
    }

    fun destroy() {
        basicAircraftControlVM = null
        virtualStickVM = null
    }

    //WAYPOINT MISSION
    private val location3DKey: DJIKey<LocationCoordinate3D> =
            FlightControllerKey.KeyAircraftLocation3D.create()

    private fun getLocation3D(): LocationCoordinate3D {
        return location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))
    }

    private val compassHeadKey: DJIKey<Double> = FlightControllerKey.KeyCompassHeading.create()
    private fun getHeading(): Double {
        return (compassHeadKey.get(0.0)).toDouble()
    }

    private var _isWaypointReached = false
    private var _isYawReached = false
    private var _isAltitudeReached = false
    private var _isIntermediaryWaypointReached = false

    // Control loop management - to prevent ghost waypoint navigation
    private var activeControlLoopHandler: Handler? = null
    private var activeControlLoopRunnable: Runnable? = null
    
    // Kill switch - when false, ALL control loops must stop immediately
    @Volatile
    private var controlLoopEnabled = false
    
    // Unique ID for each control loop session - loops check this to ensure they're still valid
    @Volatile
    private var currentControlLoopId: Long = 0
    
    // Timestamp when control loop started - used to give virtual stick time to enable
    @Volatile
    private var controlLoopStartTime: Long = 0
    
    // Grace period (ms) to allow virtual stick to enable before checking its state
    private val VIRTUAL_STICK_ENABLE_GRACE_PERIOD_MS = 1000L

    /**
     * Cancel any active control loop (gotoWP, gotoYaw, gotoAltitude, navigateTrajectory, etc.)
     * This MUST be called before starting a new control loop or when disabling virtual sticks
     */
    fun cancelActiveControlLoop() {
        // Disable control loop and increment ID to invalidate any running loops
        controlLoopEnabled = false
        currentControlLoopId++
        
        activeControlLoopRunnable?.let { runnable ->
            activeControlLoopHandler?.removeCallbacks(runnable)
        }
        activeControlLoopHandler?.removeCallbacksAndMessages(null)
        activeControlLoopHandler = null
        activeControlLoopRunnable = null
        // Reset stick to neutral position
        setStick(0F, 0F, 0F, 0F)
    }
    
    /**
     * Start a new control loop session - returns the loop ID that must be checked each iteration
     */
    private fun startNewControlLoopSession(): Long {
        cancelActiveControlLoop()  // Cancel any previous loop first
        controlLoopEnabled = true
        currentControlLoopId++
        controlLoopStartTime = System.currentTimeMillis()
        return currentControlLoopId
    }
    
    /**
     * Check if the control loop with given ID should continue running.
     * Returns false if:
     * - The control loop was explicitly cancelled (controlLoopEnabled = false)
     * - A new control loop was started (loopId doesn't match)
     * - The drone is no longer in virtual stick mode (user took manual control)
     */
    private fun shouldControlLoopContinue(loopId: Long): Boolean {
        // Check if loop was cancelled or a new one started
        if (!controlLoopEnabled || loopId != currentControlLoopId) {
            return false
        }
        
        // Give virtual stick time to enable before checking its state
        // enableVirtualStick() is async, so we need a grace period
        val timeSinceStart = System.currentTimeMillis() - controlLoopStartTime
        if (timeSinceStart < VIRTUAL_STICK_ENABLE_GRACE_PERIOD_MS) {
            // Still in grace period, don't check virtual stick state yet
            return true
        }
        
        // Check if drone is still in virtual stick mode
        // If user takes manual control, virtual stick gets disabled - kill the loop
        val isVirtualStickEnabled = virtualStickVM?.currentVirtualStickStateInfo?.value?.state?.isVirtualStickEnable ?: false
        if (!isVirtualStickEnabled) {
            // Virtual stick was disabled externally (user took control), cancel the loop
            controlLoopEnabled = false
            return false
        }
        
        return true
    }

    // Keep track of last KMZ pushed/started
    private var lastMissionNameNoExt: String = ""
    private var lastMissionKmzPath: String = ""

    // App-owned external files directory for KMZ output
    private val kmzDir: String by lazy {
        val ctx = ContextUtil.getContext()
        val base = ctx.getExternalFilesDir(null)
        val dir = File(base, "kmz").apply { mkdirs() }
        dir.absolutePath + File.separator
    }

    // STREAM STABILITY
    fun enableVirtualStick() {
        // Cancel any active control loop first to prevent ghost navigation
        cancelActiveControlLoop()
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("enableVirtualStick success.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
    }

    fun disableVirtualStick() {
        // Cancel any active control loop first
        cancelActiveControlLoop()
        virtualStickVM?.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("disableVirtualStick success.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("disableVirtualStick error,${error})")
            }
        })
    }

    /**
     * Comprehensive abort function that stops ALL types of missions/navigation:
     * 1. Cancels any active control loops (PID navigation)
     * 2. Resets virtual sticks to neutral
     * 3. Attempts to disable virtual stick (may fail if control authority was lost - that's OK)
     * 4. Stops any DJI native waypoint missions
     * 
     * This function is designed to be resilient - it will attempt all abort actions
     * regardless of individual failures, ensuring the drone stops moving.
     */
    fun abortAllMissions() {
        // 1. Cancel any active PID control loops immediately
        cancelActiveControlLoop()
        
        // 2. Reset sticks to neutral
        setStick(0F, 0F, 0F, 0F)
        
        // 3. Try to disable virtual stick (may fail if we don't have control authority - that's OK)
        virtualStickVM?.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                // Virtual stick disabled successfully
            }
            override fun onFailure(error: IDJIError) {
                // Ignore - we may not have had control authority, which is fine
                // The important thing is we've cancelled the control loops
            }
        })
        
        // 4. Also try to stop any DJI native waypoint mission
        try {
            if (lastMissionNameNoExt.isNotEmpty()) {
                WaypointMissionManager.getInstance().stopMission(lastMissionNameNoExt, object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() { }
                    override fun onFailure(error: IDJIError) { }
                })
            }
            // Also try pause in case there's an unnamed mission running
            WaypointMissionManager.getInstance().pauseMission(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { }
                override fun onFailure(error: IDJIError) { }
            })
        } catch (e: Exception) {
            // Ignore any errors - we just want to try our best to stop everything
        }
    }

    fun calculateDistance(
            latA: Double,
            lngA: Double,
            latB: Double,
            lngB: Double,
    ): Double {
        val earthR = 6371000.0
        val x =
                cos(latA * PI / 180) * cos(
                        latB * PI / 180
                ) * cos((lngA - lngB) * PI / 180)
        val y =
                sin(latA * PI / 180) * sin(
                        latB * PI / 180
                )
        var s = x + y
        if (s > 1) {
            s = 1.0
        }
        if (s < -1) {
            s = -1.0
        }
        val alpha = acos(s)
        return alpha * earthR
    }

    // Helper function to normalize an angle to the range [-180, 180]
    fun normalizeAngle(angle: Double): Double {
        var adjustedAngle = angle % 360
        if (adjustedAngle > 180) adjustedAngle -= 360
        if (adjustedAngle < -180) adjustedAngle += 360
        return adjustedAngle
    }

    fun calculateBearing(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Float {
        val lat1Rad = Math.toRadians(lat1)
        val lon1Rad = Math.toRadians(lon1)
        val lat2Rad = Math.toRadians(lat2)
        val lon2Rad = Math.toRadians(lon2)
        val deltaLon = lon2Rad - lon1Rad
        val y = sin(deltaLon) * cos(lat2Rad)
        val x = cos(lat1Rad) * sin(lat2Rad) -
                sin(lat1Rad) * cos(lat2Rad) * cos(deltaLon)
        val initialBearing = atan2(y, x)
        val initialBearingDeg = Math.toDegrees(initialBearing)
        val compassBearing = (initialBearingDeg + 360) % 360
        return compassBearing.toFloat()
    }

    fun setStick(
            leftX: Float = 0F,
            leftY: Float = 0F,
            rightX: Float = 0F,
            rightY: Float = 0F
    ) {
        virtualStickVM?.setLeftPosition(
                (leftX * Stick.MAX_STICK_POSITION_ABS).toInt(),
                (leftY * Stick.MAX_STICK_POSITION_ABS).toInt()
        )
        virtualStickVM?.setRightPosition(
                (rightX * Stick.MAX_STICK_POSITION_ABS).toInt(),
                (rightY * Stick.MAX_STICK_POSITION_ABS).toInt()
        )
    }

    fun startTakeOff() {
        // Disable virtual sticks first to ensure no control loops are running before takeoff
        disableVirtualStick()
        
        basicAircraftControlVM?.startTakeOff(object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start takeOff onSuccess.")
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("start takeOff onFailure, $error")
            }
        })
    }

    fun startLanding() {
        basicAircraftControlVM?.startLanding(object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start landing onSuccess.")
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("start landing onFailure, $error")
            }
        })
    }

    fun startReturnToHome() {
        // CRITICAL: Disable virtual stick before RTH to prevent conflicts
        // Virtual stick mode can interfere with RTH causing erratic behavior
        cancelActiveControlLoop()
        
        virtualStickVM?.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                // Virtual stick disabled, now safe to start RTH
                executeRTH()
            }

            override fun onFailure(error: IDJIError) {
                // Virtual stick may already be disabled or we don't have control authority
                // Still try RTH - the DJI SDK may handle it
                executeRTH()
            }
        })
    }
    
    private fun executeRTH() {
        basicAircraftControlVM?.startReturnToHome(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start RTH onSuccess.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("start RTH onFailure,$error")
            }
        })
    }


    private fun stopCurrentMission() {
        if (lastMissionNameNoExt.isNotEmpty()) {
            WaypointMissionManager.getInstance().stopMission(lastMissionNameNoExt, object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { /* no-op */ }
                override fun onFailure(error: IDJIError) { /* ignore */ }
            })
        } else {
            // Try to pause/stop any active mission even if we don't track the name
             WaypointMissionManager.getInstance().pauseMission(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { /* no-op */ }
                override fun onFailure(error: IDJIError) { /* ignore */ }
            })
        }
    }

    fun gotoYaw(targetYaw: Double) {
        stopCurrentMission()
        // Start new control loop session
        val loopId = startNewControlLoopSession()
        
        _isYawReached = false
        val controlLoopYaw = Handler(Looper.getMainLooper())
        val updateInterval = 100.0 // Update every 100 ms
        val maxYawRate = 30.0 // degrees per second
        val yawPID = PID(3.0, 0.0, 0.0, updateInterval/1000, -maxYawRate to maxYawRate)

        virtualStickVM?.enableVirtualStickAdvancedMode()
        // Enable Virtual Stick and advanced mode
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()

        val runnable = object : Runnable {
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }
                
                val currentPosition = getLocation3D()
                val currentYaw = getHeading()
                val yawError = normalizeAngle(targetYaw - currentYaw)
                val angularVelocity = yawPID.update(yawError)

                // Stop if the error is within a threshold
                if (abs(yawError) < 0.5) {
                    setStick(0F, 0F, 0F, 0F)
                    _isYawReached = true
                    controlLoopEnabled = false
                    return
                }

                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch = 0.0
                    this.roll = 0.0
                    this.yaw = angularVelocity
                    this.verticalThrottle = currentPosition.altitude
                    this.verticalControlMode = VerticalControlMode.POSITION
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                virtualStickVM?.sendVirtualStickAdvancedParam(flightControlParam)
                controlLoopYaw.postDelayed(this, updateInterval.toLong())
            }
        }
        
        // Store references to allow cancellation
        activeControlLoopHandler = controlLoopYaw
        activeControlLoopRunnable = runnable
        controlLoopYaw.post(runnable)
    }

    fun gotoAltitude(targetAltitude: Double) {
        stopCurrentMission()
        // Start new control loop session
        val loopId = startNewControlLoopSession()

        // Enable Virtual Stick and advanced mode
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()

        _isAltitudeReached = false
        val controlLoopHandler = Handler(Looper.getMainLooper())
        val updateInterval = 100L // Update every 100 ms
        
        // Capture initial yaw ONCE to prevent oscillation from compass noise
        val initialYaw = getHeading()

        // Enable advanced Virtual Stick mode
        virtualStickVM?.enableVirtualStickAdvancedMode()

        val runnable = object : Runnable {
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }

                val currentPosition = getLocation3D()
                val altitudeError = targetAltitude - currentPosition.altitude
                val distanceToAltitude = abs(altitudeError)

                if (distanceToAltitude < 0.4) { // Stop if close enough to the target altitude
                    setStick(0F, 0F, 0F, 0F)
                    _isAltitudeReached = true
                    controlLoopEnabled = false
                    return
                }

                // Proportional gain
                val Kp = 0.5 // Adjust this gain as needed

                // Calculate the vertical speed command
                var verticalSpeed = Kp * altitudeError

                // Limit the vertical speed to the maximum allowed by the drone
                val maxVerticalSpeed = 4.0 // Maximum vertical speed in m/s
                verticalSpeed = verticalSpeed.coerceIn(-maxVerticalSpeed, maxVerticalSpeed)

                // Use initial yaw captured at start to prevent oscillation from compass noise
                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch = 0.0
                    this.roll = 0.0
                    this.yaw = initialYaw
                    this.verticalThrottle = verticalSpeed
                    this.verticalControlMode = VerticalControlMode.VELOCITY
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGLE
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                virtualStickVM?.sendVirtualStickAdvancedParam(flightControlParam)

                // Schedule the next update
                controlLoopHandler.postDelayed(this, updateInterval)
            }
        }
        
        // Store references to allow cancellation
        activeControlLoopHandler = controlLoopHandler
        activeControlLoopRunnable = runnable
        controlLoopHandler.post(runnable)
    }

    fun gotoWP(targetLatitude: Double, targetLongitude: Double, targetAltitude: Double) {
        stopCurrentMission()
        // Start new control loop session
        val loopId = startNewControlLoopSession()
        
        val controlLoop = Handler(Looper.getMainLooper())
        val updateInterval: Long = 100 // Update every 100 ms

        // Enable Virtual Stick and advanced mode
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        _isWaypointReached = false
        virtualStickVM?.enableVirtualStickAdvancedMode()
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()

        val runnable = object : Runnable {
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }

                val currentPosition = getLocation3D()
                val distanceToWaypoint = calculateDistance(
                        targetLatitude,
                        targetLongitude,
                        currentPosition.latitude,
                        currentPosition.longitude
                )

                val altError = targetAltitude - currentPosition.altitude

                if (distanceToWaypoint < 0.5 && abs(altError) < 0.5) { // Stop if close enough to the waypoint
                    setStick(0F, 0F, 0F, 0F)
                    _isWaypointReached = true
                    controlLoopEnabled = false
                    return
                }
                // Calculate the desired yaw angle to face the waypoint
                val desiredYaw = calculateBearing(
                        currentPosition.latitude,
                        currentPosition.longitude,
                        targetLatitude,
                        targetLongitude
                ).toDouble()

                val adjustedDesiredYaw = if (desiredYaw > 180) desiredYaw - 360 else desiredYaw

                // Get the current yaw angle of the drone
                val currentYaw = getHeading()

                // Compute yaw error
                var yawError = adjustedDesiredYaw - currentYaw
                yawError = normalizeAngle(yawError)

                // Set yaw_control to the desired yaw angle
                val yawControl = adjustedDesiredYaw

                // Compute forward speed proportional to the distance to the waypoint
                val maxSpeed = 5f // Maximum speed in m/s
                val kp = 0.5f // Proportional gain

                var speed = (kp * distanceToWaypoint).toFloat()

                if (speed > maxSpeed) {
                    speed = maxSpeed
                }

                // Reduce speed if the drone is not facing the waypoint
                val maxYawError = 15f // degrees
                val yawErrorFactor = max(0f, 1f - (abs(yawError) / maxYawError).toFloat())
                speed *= yawErrorFactor

                // Set pitch_control to move forward at the computed speed
                val pitchControl = speed.toDouble()

                // Set roll_control to zero (no lateral movement)
                val rollControl = 0F.toDouble()

                // Create the VirtualStickFlightControlParam object
                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch =
                            rollControl // Weird, it only works if I'm switching the pitch and roll (I think it's a bug, or it's because I fly in mode 1 ?)
                    this.roll = pitchControl
                    this.yaw = yawControl
                    this.verticalThrottle = targetAltitude
                    this.verticalControlMode = VerticalControlMode.POSITION
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGLE
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                // Send the virtual stick control data
                virtualStickVM?.sendVirtualStickAdvancedParam(flightControlParam)
                // Schedule the next update
                controlLoop.postDelayed(this, updateInterval)
            }
        }
        
        // Store references to allow cancellation
        activeControlLoopHandler = controlLoop
        activeControlLoopRunnable = runnable
        controlLoop.post(runnable)
    }

    fun navigateToWaypointWithPID(targetLatitude: Double, targetLongitude: Double, targetAlt: Double, targetYaw: Double, maxSpeed: Double) {
        stopCurrentMission()
        // Start new control loop session
        val loopId = startNewControlLoopSession()

        val updateInterval = 100.0  // Update every 100 ms
        val maxYawRate = 30.0 // degrees per second

        virtualStickVM?.enableVirtualStickAdvancedMode()
        // Enable Virtual Stick and advanced mode
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()

        val distancePID = PID(0.65, 0.0001, 0.001, updateInterval/1000, 0.0 to maxSpeed)
        val yawPID = PID(3.0, 0.0000, 0.00, updateInterval/1000, -maxYawRate to maxYawRate)

        val controlLoop = Handler(Looper.getMainLooper())

        _isWaypointReached = false
        virtualStickVM?.enableVirtualStickAdvancedMode()


        val runnable = object : Runnable {
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }
                
                val currentPosition = getLocation3D()
                val currentYaw = getHeading()

                val distance = calculateDistance(targetLatitude, targetLongitude, currentPosition.latitude, currentPosition.longitude)
                val targetSpeed = distancePID.update(distance)
                val movementDirection = calculateBearing(currentPosition.latitude, currentPosition.longitude, targetLatitude, targetLongitude).toDouble()

                val yawError = normalizeAngle(targetYaw - currentYaw)
                val angularVelocity = yawPID.update(yawError)

                val movementDirectionRelative = normalizeAngle(movementDirection - currentYaw) // Relative to the drone's heading
                val pitch = targetSpeed * cos(Math.toRadians(movementDirectionRelative))
                val roll = targetSpeed * sin(Math.toRadians(movementDirectionRelative))

                val altError = targetAlt - currentPosition.altitude

                if (distance < 2 && abs(yawError) < 4 && abs(altError) < 2) { // Stop if close enough to the waypoint
                    setStick(0F, 0F, 0F, 0F)
                    _isWaypointReached = true
                    controlLoopEnabled = false
                    disableVirtualStick() // Disable virtual stick to let drone hold GPS position
                    return
                }

                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch = roll // Weird, it only works if I'm switching the pitch and roll (I think it's a bug, or it's because I fly in mode 1 ?)
                    this.roll = pitch
                    this.yaw = angularVelocity
                    this.verticalThrottle = targetAlt
                    this.verticalControlMode = VerticalControlMode.POSITION
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                virtualStickVM?.sendVirtualStickAdvancedParam(flightControlParam)
                controlLoop.postDelayed(this, updateInterval.toLong())
            }
        }
        
        // Store references to allow cancellation
        activeControlLoopHandler = controlLoop
        activeControlLoopRunnable = runnable
        controlLoop.post(runnable)
    }

    fun navigateTrajectory(
        waypoints: List<Triple<Double, Double, Double>>,
        lookaheadDistance: Double = 5.5,
        cruiseSpeed: Double = 5.0,
        minSpeedFinal: Double = 1.0,
        slowdownRadius: Double = 4.0
    ) {
        stopCurrentMission()
        if (waypoints.size < 2) return
        
        // Start new control loop session
        val loopId = startNewControlLoopSession()

        val updateIntervalMs = 100L

        var currentIndex = 0
        _isWaypointReached = false
        _isIntermediaryWaypointReached = false

        virtualStickVM?.enableVirtualStickAdvancedMode()
        // Enable Virtual Stick and advanced mode
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()
        val controlLoop = Handler(Looper.getMainLooper())

        // Helper: Compute great-circle distance (meters) between two lat/lon
        fun calculateDistance(latA: Double, lonA: Double, latB: Double, lonB: Double): Double {
            val earthR = 6371000.0
            val phi1 = Math.toRadians(latA)
            val phi2 = Math.toRadians(latB)
            val deltaPhi = Math.toRadians(latB - latA)
            val deltaLambda = Math.toRadians(lonB - lonA)
            val a = sin(deltaPhi/2) * sin(deltaPhi/2) +
                    cos(phi1) * cos(phi2) *
                    sin(deltaLambda/2) * sin(deltaLambda/2)
            val c = 2 * atan2(sqrt(a), sqrt(1 - a))
            return earthR * c
        }

        // Helper: Compute bearing from (lat1, lon1) to (lat2, lon2)
        fun calculateBearing(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
            val phi1 = Math.toRadians(lat1)
            val phi2 = Math.toRadians(lat2)
            val deltaLambda = Math.toRadians(lon2 - lon1)
            val y = sin(deltaLambda) * cos(phi2)
            val x = cos(phi1) * sin(phi2) -
                    sin(phi1) * cos(phi2) * cos(deltaLambda)
            val bearing = Math.toDegrees(atan2(y, x))
            return (bearing + 360) % 360
        }

        // Helper: Normalize angle to [-180, 180]
        fun normalizeAngle(angle: Double): Double {
            var a = angle % 360.0
            if (a > 180.0) a -= 360.0
            if (a < -180.0) a += 360.0
            return a
        }

        // Helper: Progress along [A,B] segment (0=start, 1=end, >1=after end)
        fun progressOnSegment(
            A: Triple<Double, Double, Double>,
            B: Triple<Double, Double, Double>,
            pos: LocationCoordinate3D
        ): Double {
            val ax = A.first; val ay = A.second
            val bx = B.first; val by = B.second
            val px = pos.latitude; val py = pos.longitude
            val dx = bx - ax; val dy = by - ay
            val segLen2 = dx*dx + dy*dy
            if (segLen2 == 0.0) return 0.0
            val dot = ((px - ax) * dx + (py - ay) * dy)
            return dot / segLen2 // 0=start, 1=end, >1=after end
        }

        val runnable = object : Runnable {
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }
                
                val current = getLocation3D()
                val currentYaw = getHeading()

                // Segment indices
                val idxA = currentIndex
                val idxB = (currentIndex + 1).coerceAtMost(waypoints.lastIndex)
                val start = waypoints[idxA]
                val end = waypoints[idxB]

                // Progress along the segment [start, end]
                val progress = progressOnSegment(start, end, current)
                // Project drone onto the segment [start, end]
                val segLen = calculateDistance(start.first, start.second, end.first, end.second)
                val projRatio = progress.coerceIn(0.0, 1.0)

                // Pure pursuit: lookahead point further along the segment
                val lookaheadRatio = ((segLen * projRatio) + lookaheadDistance) / segLen
                val lookaheadRatioClamped = lookaheadRatio.coerceIn(0.0, 1.0)
                val lookahead = Triple(
                    start.first + (end.first - start.first) * lookaheadRatioClamped,
                    start.second + (end.second - start.second) * lookaheadRatioClamped,
                    start.third + (end.third - start.third) * lookaheadRatioClamped
                )

                // Target altitude is smooth
                val targetAlt = lookahead.third

                // --- Yaw Control: P controller for angular velocity ---
                val targetYaw = calculateBearing(current.latitude, current.longitude, lookahead.first, lookahead.second)
                val yawError = normalizeAngle(targetYaw - currentYaw)
                val Kp_yaw = 1.0 // Tune as needed; 1.0 = 1 deg/s per deg error
                val maxYawRate = 30.0 // degrees/sec, DJI safe max
                val targetYawRate = (Kp_yaw * yawError).coerceIn(-maxYawRate, maxYawRate)

                // Move toward lookahead
                val moveDir = targetYaw
                val moveDirRel = normalizeAngle(moveDir - currentYaw)
                var targetSpeed = cruiseSpeed

                // Last segment: slow down as you approach the last waypoint
                val isLastSegment = idxB == waypoints.lastIndex
                if (isLastSegment) {
                    val distToEnd = calculateDistance(current.latitude, current.longitude, end.first, end.second)
                    if (distToEnd < slowdownRadius)
                        targetSpeed = minSpeedFinal + (cruiseSpeed - minSpeedFinal) * (distToEnd / slowdownRadius)
                }

                val pitch = targetSpeed * cos(Math.toRadians(moveDirRel))
                val roll = targetSpeed * sin(Math.toRadians(moveDirRel))

                // Stop criteria: last segment, close to endpoint, and altitude close
                val reached = isLastSegment &&
                        (calculateDistance(current.latitude, current.longitude, end.first, end.second) < 0.8) &&
                        (abs(targetAlt - current.altitude) < 1.0)

                if (reached) {
                    setStick(0F, 0F, 0F, 0F)
                    _isWaypointReached = true
                    controlLoopEnabled = false
                    return
                }

                // Passed the end of the segment: go to next
                if (!isLastSegment && progress > 1.0) {
                    currentIndex++
                    controlLoop.postDelayed(this, updateIntervalMs)
                    return
                }

                // Send control command
                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch = roll // DJI SDK: roll/pitch swapped
                    this.roll = pitch
                    this.yaw = targetYawRate
                    this.verticalThrottle = targetAlt
                    this.verticalControlMode = VerticalControlMode.POSITION
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                virtualStickVM?.sendVirtualStickAdvancedParam(flightControlParam)
                controlLoop.postDelayed(this, updateIntervalMs)
            }
        }
        
        // Store references to allow cancellation
        activeControlLoopHandler = controlLoop
        activeControlLoopRunnable = runnable
        controlLoop.post(runnable)
    }

    // === DJI Native Wayline (KMZ) flow ===
    fun generateTrajectoryName(): String {
        val dateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault())
        return "trajectory_${dateFormat.format(Date())}"
    }

    fun getKmzDirectory(): String = kmzDir

    fun getLastMissionNameNoExt(): String = lastMissionNameNoExt
    
    fun getLastMissionKmzPath(): String = lastMissionKmzPath

    /**
     * Create a waypoint model from lat/lon/height with gimbal pitch set to -90 (looking down)
     */
    fun createWaypointFromLatLon(
        lat: Double,
        lon: Double,
        heightMeters: Double,
        index: Int
    ): WaypointInfoModel {
        val waypointInfo = WaypointInfoModel()
        val waypoint = WaylineWaypoint()

        val coordinate2D = WaylineLocationCoordinate2D().apply {
            latitude = lat
            longitude = lon
        }
        waypoint.location = coordinate2D
        waypoint.waypointIndex = index
        waypoint.height = heightMeters
        waypoint.ellipsoidHeight = heightMeters
        waypoint.useGlobalFlightHeight = false

        waypoint.useGlobalAutoFlightSpeed = true
        waypoint.useGlobalTurnParam = true

        val yawParam = WaylineWaypointYawParam().apply {
            yawMode = WaylineWaypointYawMode.FOLLOW_WAYLINE
            yawPathMode = WaylineWaypointYawPathMode.FOLLOW_BAD_ARC
            poiLocation = WaylineLocationCoordinate3D(lat, lon, heightMeters)
        }
        waypoint.yawParam = yawParam
        waypoint.useGlobalYawParam = false
        waypoint.isWaylineWaypointYawParamSet = true

        // Set gimbal pitch angle directly on the waypoint
        waypoint.gimbalPitchAngle = -90.0  // Gimbal looking straight down during trajectory following

        // Use global gimbal heading param (set at template level)
        waypoint.useGlobalGimbalHeadingParam = true

        waypointInfo.waylineWaypoint = waypoint

        // Create gimbal rotate action to set pitch to -90 degrees (looking straight down)
        val gimbalRotateParam = ActionGimbalRotateParam()
        gimbalRotateParam.enablePitch = true
        gimbalRotateParam.pitch = -90.0  // Look straight down
        gimbalRotateParam.rotateMode = WaylineGimbalActuatorRotateMode.ABSOLUTE_ANGLE
        gimbalRotateParam.payloadPositionIndex = 0
        
        val gimbalAction = WaylineActionInfo()
        gimbalAction.actionType = WaylineActionType.GIMBAL_ROTATE
        gimbalAction.gimbalRotateParam = gimbalRotateParam
        
        waypointInfo.actionInfos = arrayListOf(gimbalAction)
        return waypointInfo
    }

    fun createWaylineMission(): WaylineMission {
        val m = WaylineMission()
        val now = System.currentTimeMillis().toDouble()
        m.createTime = now
        m.updateTime = now
        return m
    }

    fun createMissionConfig(
        finishAction: WaylineFinishedAction = WaylineFinishedAction.NO_ACTION,
        lostAction: WaylineExitOnRCLostAction = WaylineExitOnRCLostAction.GO_BACK
    ): WaylineMissionConfig {
        val c = WaylineMissionConfig()
        c.flyToWaylineMode = WaylineFlyToWaylineMode.SAFELY
        c.finishAction = finishAction
        c.droneInfo = WaylineDroneInfo()
        c.securityTakeOffHeight = 20.0
        c.isSecurityTakeOffHeightSet = true
        c.exitOnRCLostBehavior = WaylineExitOnRCLostBehavior.EXCUTE_RC_LOST_ACTION
        c.exitOnRCLostType = lostAction
        c.globalTransitionalSpeed = 10.0
        c.payloadInfo = ArrayList()
        return c
    }

    private fun createTemplateWaypointInfo(
        waypointInfoModels: List<WaypointInfoModel>
    ): WaylineTemplateWaypointInfo {
        val waypoints = waypointInfoModels.map { it.waylineWaypoint }
        val info = WaylineTemplateWaypointInfo()
        info.waypoints = waypoints
        info.actionGroups = transformActionsToGroups(waypointInfoModels)  // Build proper action groups
        info.globalFlightHeight = 100.0
        info.isGlobalFlightHeightSet = true
        info.globalTurnMode = WaylineWaypointTurnMode.TO_POINT_AND_STOP_WITH_DISCONTINUITY_CURVATURE
        info.useStraightLine = true
        info.isTemplateGlobalTurnModeSet = true

        val poi = if (waypoints.isNotEmpty()) {
            val first = waypoints.first()
            first.yawParam?.poiLocation
                ?: WaylineLocationCoordinate3D(first.location.latitude, first.location.longitude, first.height)
        } else WaylineLocationCoordinate3D(0.0, 0.0, 0.0)

        val yawParam = WaylineWaypointYawParam().apply {
            yawMode = WaylineWaypointYawMode.FOLLOW_WAYLINE
            poiLocation = poi
        }
        info.globalYawParam = yawParam
        info.isTemplateGlobalYawParamSet = true
        
        // Set global gimbal heading param to look straight down (-90 degrees pitch)
        val globalGimbalParam = WaylineWaypointGimbalHeadingParam()
        globalGimbalParam.headingMode = WaylineWaypointGimbalHeadingMode.find(0)
        globalGimbalParam.pitchAngle = -90.0  // Look straight down
        info.globalGimbalHeadingParam = globalGimbalParam
        info.isTemplateGlobalGimbalHeadingParamSet = true
        
        info.pitchMode = WaylineWaypointPitchMode.USE_POINT_SETTING  // Use point setting to apply gimbal pitch
        return info
    }

    // Transform waypoint actions into proper action groups for KMZ
    private fun transformActionsToGroups(waypointInfoModels: List<WaypointInfoModel>): ArrayList<WaylineActionGroup> {
        val actionGroups = ArrayList<WaylineActionGroup>()
        
        for (i in waypointInfoModels.indices) {
            val actionInfos = waypointInfoModels[i].actionInfos
            if (actionInfos.isNotEmpty()) {
                val actionGroup = WaylineActionGroup()
                
                // Set trigger to execute when reaching waypoint
                val trigger = WaylineActionTrigger()
                trigger.setTriggerType(WaylineActionTriggerType.REACH_POINT)
                actionGroup.setTrigger(trigger)
                
                actionGroup.setGroupId(actionGroups.size)
                actionGroup.setStartIndex(i)
                actionGroup.setEndIndex(i)
                actionGroup.setActions(actionInfos)
                
                // Build action tree structure
                val nodeLists = ArrayList<WaylineActionNodeList>()
                
                // Root node
                val root = WaylineActionNodeList()
                val treeNodes = ArrayList<WaylineActionTreeNode>()
                val rootNode = WaylineActionTreeNode()
                rootNode.setNodeType(WaylineActionsRelationType.SEQUENCE)
                rootNode.setChildrenNum(actionInfos.size)
                treeNodes.add(rootNode)
                root.setNodes(treeNodes)
                nodeLists.add(root)
                
                // Children nodes (one for each action)
                val children = WaylineActionNodeList()
                val childrenNodeList = ArrayList<WaylineActionTreeNode>()
                for (j in actionInfos.indices) {
                    val child = WaylineActionTreeNode()
                    child.setNodeType(WaylineActionsRelationType.LEAF)
                    child.setActionIndex(j)
                    childrenNodeList.add(child)
                }
                children.setNodes(childrenNodeList)
                nodeLists.add(children)
                
                actionGroup.setNodeLists(nodeLists)
                actionGroups.add(actionGroup)
            }
        }
        
        return actionGroups
    }

    fun createTemplate(
        waypointInfoModels: List<WaypointInfoModel>,
        trajectorySpeed: Double = 5.0
    ): Template {
        val t = Template()
        t.waypointInfo = createTemplateWaypointInfo(waypointInfoModels)

        val cp = WaylineCoordinateParam().apply {
            coordinateMode = WaylineCoordinateMode.WGS84
            positioningType = WaylinePositioningType.GPS
            isWaylinePositioningTypeSet = true
            altitudeMode = WaylineAltitudeMode.RELATIVE_TO_START_POINT
        }
        t.coordinateParam = cp
        t.useGlobalTransitionalSpeed = true
        t.autoFlightSpeed = trajectorySpeed
        t.payloadParam = ArrayList()
        return t
    }

    fun extractWaylineIdsFromKmz(kmzPath: String): ArrayList<Int> {
        val result = arrayListOf<Int>()
        runCatching {
            ZipFile(File(kmzPath)).use { zip ->
                val entry: ZipEntry? = zip.getEntry("wpmz/waylines.wpml")
                if (entry != null) {
                    val text = zip.getInputStream(entry).use { it.readBytes().toString(Charsets.UTF_8) }
                    val regex = Regex("<\\s*wpml:waylineId\\s*>\\s*([0-9]+)\\s*<\\s*/\\s*wpml:waylineId\\s*>")
                    regex.findAll(text).forEach { m ->
                        m.groupValues.getOrNull(1)?.toIntOrNull()?.let { result.add(it) }
                    }
                }
            }
        }
        return result
    }

    /**
     * Generate and save a KMZ file from waypoint models
     * Returns the path to the saved KMZ file
     */
    fun generateAndSaveKmz(
        waypointInfoModels: List<WaypointInfoModel>,
        missionName: String = generateTrajectoryName(),
        trajectorySpeed: Double = 5.0,
        finishAction: WaylineFinishedAction = WaylineFinishedAction.GO_HOME,
        lostAction: WaylineExitOnRCLostAction = WaylineExitOnRCLostAction.GO_BACK
    ): String {
        WPMZManager.getInstance().init(ContextUtil.getContext())
        
        val waylineMission = createWaylineMission()
        val missionConfig = createMissionConfig(finishAction, lostAction)
        val template = createTemplate(waypointInfoModels, trajectorySpeed)
        
        val kmzOutPath = kmzDir + missionName + ".kmz"
        WPMZManager.getInstance().generateKMZFile(kmzOutPath, waylineMission, missionConfig, template)
        
        lastMissionNameNoExt = missionName
        lastMissionKmzPath = kmzOutPath
        
        return kmzOutPath
    }

    /**
     * Push a KMZ file to the aircraft
     */
    fun pushKmzToAircraft(
        kmzPath: String,
        onProgress: ((Double) -> Unit)? = null,
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        lastMissionKmzPath = kmzPath
        lastMissionNameNoExt = File(kmzPath).nameWithoutExtension
        
        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzPath, object :
            CommonCallbacks.CompletionCallbackWithProgress<Double> {
            override fun onProgressUpdate(progress: Double) {
                onProgress?.invoke(progress)
            }
            override fun onSuccess() {
                onSuccess()
            }
            override fun onFailure(error: IDJIError) {
                onFailure(error)
            }
        })
    }

    /**
     * Start a mission that has been pushed to the aircraft
     */
    fun startMission(
        missionNameNoExt: String = lastMissionNameNoExt,
        kmzPath: String = lastMissionKmzPath,
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        if (missionNameNoExt.isEmpty()) {
            // Create a simple error without using ErrorType enum
            val noMissionError = object : IDJIError {
                override fun errorType() = null
                override fun errorCode() = "NO_MISSION"
                override fun description() = "No mission loaded"
                override fun isError(p0: String?) = true
                override fun innerCode() = "NO_MISSION"
                override fun hint() = "Load a mission first"
            }
            onFailure(noMissionError)
            return
        }
        
        val ids = extractWaylineIdsFromKmz(kmzPath).ifEmpty { arrayListOf(0) }
        WaypointMissionManager.getInstance().startMission(
            missionNameNoExt,
            ids,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    onSuccess()
                }
                override fun onFailure(error: IDJIError) {
                    onFailure(error)
                }
            }
        )
    }

    /**
     * Pause the current mission
     */
    fun pauseMission(
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        WaypointMissionManager.getInstance().pauseMission(object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                onSuccess()
            }
            override fun onFailure(error: IDJIError) {
                onFailure(error)
            }
        })
    }

    /**
     * Stop the current mission
     */
    fun stopMission(
        missionNameNoExt: String = lastMissionNameNoExt,
        onSuccess: () -> Unit,
        onFailure: (IDJIError) -> Unit
    ) {
        if (missionNameNoExt.isEmpty()) {
            pauseMission(onSuccess, onFailure)
            return
        }
        WaypointMissionManager.getInstance().stopMission(missionNameNoExt, object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                onSuccess()
            }
            override fun onFailure(error: IDJIError) {
                onFailure(error)
            }
        })
    }

    fun navigateTrajectoryNative(
        userWaypoints: List<Triple<Double, Double, Double>>,
        trajectorySpeed: Double
    ) {
        if (userWaypoints.size < 2) {
            ToastUtils.showToast("Need at least 2 waypoints")
            return
        }

        // Attempt to stop any previous mission we started
        if (lastMissionNameNoExt.isNotEmpty()) {
            WaypointMissionManager.getInstance().stopMission(lastMissionNameNoExt, object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { /* no-op */ }
                override fun onFailure(error: IDJIError) { /* ignore */ }
            })
        }

        // Init WPMZ (idempotent)
        WPMZManager.getInstance().init(ContextUtil.getContext())

        // Build waypoints
        val wpModels = ArrayList<WaypointInfoModel>()
        userWaypoints.forEachIndexed { idx, t ->
            wpModels.add(createWaypointFromLatLon(t.first, t.second, t.third, idx))
        }

        // Build mission components
        val mission = createWaylineMission()
        val config = createMissionConfig()
        val template = createTemplate(wpModels, trajectorySpeed)

        // Generate KMZ
        val missionName = generateTrajectoryName()
        val kmzOutPath = "$kmzDir$missionName.kmz"
        WPMZManager.getInstance().generateKMZFile(kmzOutPath, mission, config, template)

        lastMissionNameNoExt = missionName
        lastMissionKmzPath = kmzOutPath

        // Push to aircraft then start
        WaypointMissionManager.getInstance().pushKMZFileToAircraft(kmzOutPath, object :
            CommonCallbacks.CompletionCallbackWithProgress<Double> {
            override fun onProgressUpdate(progress: Double) {
                // optional: progress log
            }
            override fun onSuccess() {
                val ids = extractWaylineIdsFromKmz(kmzOutPath).ifEmpty { arrayListOf(0) }
                WaypointMissionManager.getInstance().startMission(
                    lastMissionNameNoExt,
                    ids,
                    object : CommonCallbacks.CompletionCallback {
                        override fun onSuccess() {
                            ToastUtils.showToast("Mission started: $lastMissionNameNoExt")
                        }
                        override fun onFailure(error: IDJIError) {
                            ToastUtils.showToast("Start mission failed: ${error.description()}")
                        }
                    }
                )
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("Push KMZ failed: ${error.description()}")
            }
        })
    }

    fun endMission() {
        if (lastMissionNameNoExt.isEmpty()) {
            // Try to pause anyway
            WaypointMissionManager.getInstance().pauseMission(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { ToastUtils.showToast("Mission paused") }
                override fun onFailure(error: IDJIError) { ToastUtils.showToast("No mission to stop") }
            })
            return
        }
        WaypointMissionManager.getInstance().stopMission(lastMissionNameNoExt, object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("Mission stopped: $lastMissionNameNoExt")
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("Stop mission failed: ${error.description()}")
            }
        })
    }

    // Getter pour isWaypointReached
    fun isWaypointReached(): Boolean {
        return _isWaypointReached
    }

    // Getter pour isYawReached
    fun isYawReached(): Boolean {
        return _isYawReached
    }

    // Idem pour isAltitudeReached, etc.
    fun isAltitudeReached(): Boolean {
        return _isAltitudeReached
    }

    fun isIntermediaryWaypointReached(): Boolean {
        return _isIntermediaryWaypointReached
    }

    private val goHomeHeightKey: DJIKey<Int> = FlightControllerKey.KeyGoHomeHeight.create()

    fun setRTHAltitude(altitude: Int) {
        goHomeHeightKey.set(altitude)
        ToastUtils.showToast("RTH altitude set to $altitude m")
    }
}
