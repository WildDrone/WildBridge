package dji.sampleV5.moduleaircraft.controller

import android.os.Handler
import android.os.Looper
import dji.sampleV5.moduleaircraft.models.BasicAircraftControlVM
import dji.sampleV5.moduleaircraft.models.SimulatorVM
import dji.sampleV5.moduleaircraft.models.VirtualStickVM
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.virtualstick.Stick
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.flightcontroller.*
import dji.sampleV5.modulecommon.util.ToastUtils
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.v5.et.create
import dji.v5.et.get
import kotlin.math.*
import dji.sampleV5.modulecommon.models.LiveStreamVM


object DroneController {

    lateinit var basicAircraftControlVM: BasicAircraftControlVM
    lateinit var virtualStickVM: VirtualStickVM
    lateinit var simulatorVM: SimulatorVM
    lateinit var LiveStreamVM: LiveStreamVM

    fun init(basicVM: BasicAircraftControlVM, stickVM: VirtualStickVM, simVM: SimulatorVM, liveVM: LiveStreamVM ) {
        basicAircraftControlVM = basicVM
        virtualStickVM = stickVM
        simulatorVM = simVM
        LiveStreamVM = liveVM
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

    private var isWaypointReached = false
    private var isYawReached = false
    private var isAltitudeReached = false
    private var isIntermediaryWaypointReached = false

    // STREAM STABILITY
    fun enableVirtualStick() {
        virtualStickVM.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("enableVirtualStick success.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
    }

    fun disableVirtualStick() {
        virtualStickVM.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("disableVirtualStick success.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("disableVirtualStick error,${error})")
            }
        })
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
        virtualStickVM.setLeftPosition(
                (leftX * Stick.MAX_STICK_POSITION_ABS).toInt(),
                (leftY * Stick.MAX_STICK_POSITION_ABS).toInt()
        )
        virtualStickVM.setRightPosition(
                (rightX * Stick.MAX_STICK_POSITION_ABS).toInt(),
                (rightY * Stick.MAX_STICK_POSITION_ABS).toInt()
        )
    }

    fun startTakeOff() {
        basicAircraftControlVM.startTakeOff(object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start takeOff onSuccess.")
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("start takeOff onFailure, $error")
            }
        })
    }

    fun startLanding() {
        basicAircraftControlVM.startLanding(object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start landing onSuccess.")
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("start landing onFailure, $error")
            }
        })
    }

    fun startReturnToHome() {
        basicAircraftControlVM.startReturnToHome(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start RTH onSuccess.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("start RTH onFailure,$error")
            }
        })
    }


    fun gotoYaw(targetYaw: Double) {
        isYawReached = false
        val controlLoopYaw = Handler(Looper.getMainLooper())
        val updateInterval = 100.0 // Update every 100 ms
        val yawPID = PID(3.0, 0.0, 0.0, updateInterval/1000, -30.0 to 30.0)

        // Enable virtual stick with function defined before
        virtualStickVM.enableVirtualStickAdvancedMode()
        val flightControlParam = VirtualStickFlightControlParam().apply {
            this.pitch = 0.0
            this.roll = 0.0
            this.verticalThrottle = 0.0
            this.verticalControlMode = VerticalControlMode.POSITION
            this.rollPitchControlMode = RollPitchControlMode.VELOCITY
            this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
            this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
        }

        controlLoopYaw.post(object : Runnable {
            override fun run() {
                // Normalize target yaw to [-180, 180] range
                val adjustedDesiredYaw = normalizeAngle(targetYaw)

                // Get the current yaw angle of the drone
                val currentYaw = getHeading()

                // Compute yaw error and normalize within [-180, 180]
                var yawError = adjustedDesiredYaw - currentYaw
                yawError = normalizeAngle(yawError)

                // Stop if the error is within a threshold
                if (abs(yawError) < 0.5) { // Stop if close enough to the target yaw
                    isYawReached = true
                    return
                }

                // Calculate angular velocity using PID
                val angularVelocity = yawPID.update(yawError)

                // Set yaw in the control parameters and send it
                flightControlParam.yaw = angularVelocity
                virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)

                // Schedule the next update
                controlLoopYaw.postDelayed(this, updateInterval.toLong())
            }
        })
    }

    fun gotoAltitude(targetAltitude: Double) {
        isAltitudeReached = false
        val controlLoopHandler = Handler(Looper.getMainLooper())
        val updateInterval = 100L // Update every 100 ms

        // Enable advanced Virtual Stick mode
        virtualStickVM.enableVirtualStickAdvancedMode()

        controlLoopHandler.post(object : Runnable {
            override fun run() {

                val currentPosition = getLocation3D()
                val altitudeError = targetAltitude - currentPosition.altitude
                val distanceToAltitude = abs(altitudeError)

                if (distanceToAltitude < 0.4) { // Stop if close enough to the target altitude
                    setStick(0F, 0F, 0F, 0F)
                    isAltitudeReached = true
                    return
                }

                // Proportional gain
                val Kp = 0.5 // Adjust this gain as needed

                // Calculate the vertical speed command
                var verticalSpeed = Kp * altitudeError

                // Limit the vertical speed to the maximum allowed by the drone
                val maxVerticalSpeed = 4.0 // Maximum vertical speed in m/s
                verticalSpeed = verticalSpeed.coerceIn(-maxVerticalSpeed, maxVerticalSpeed)

                val currentYaw = getHeading()

                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch = 0.0
                    this.roll = 0.0
                    this.yaw = currentYaw
                    this.verticalThrottle = verticalSpeed
                    this.verticalControlMode = VerticalControlMode.VELOCITY
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGLE
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)

                // Schedule the next update
                controlLoopHandler.postDelayed(this, updateInterval)
            }
        })
    }

    fun gotoWP(targetLatitude: Double, targetLongitude: Double, targetAltitude: Double) {
        val controlLoop = Handler(Looper.getMainLooper())
        val updateInterval: Long = 100 // Update every 100 ms

        // Enable virtual stick with function defined before
        isWaypointReached = false
        virtualStickVM.enableVirtualStickAdvancedMode()

        controlLoop.post(object : Runnable {
            override fun run() {

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
                    isWaypointReached = true
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
                virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)
                // Schedule the next update
                controlLoop.postDelayed(this, updateInterval)
            }
        })
    }

    fun navigateToWaypointWithPID(targetLatitude: Double, targetLongitude: Double, targetAlt: Double, targetYaw: Double) {

        val updateInterval = 100.0  // Update every 100 ms
        val maxSpeed = 5.0 // meters per second
        val maxYawRate = 30.0 // degrees per second

        virtualStickVM.enableVirtualStickAdvancedMode()

        val distancePID = PID(0.65, 0.0001, 0.001, updateInterval/1000, 0.0 to maxSpeed)
        val yawPID = PID(3.0, 0.0000, 0.00, updateInterval/1000, -maxYawRate to maxYawRate)

        val controlLoop = Handler(Looper.getMainLooper())

        isWaypointReached = false
        virtualStickVM.enableVirtualStickAdvancedMode()

        controlLoop.post(object : Runnable {
            override fun run() {
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
                    isWaypointReached = true
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

                virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)
                controlLoop.postDelayed(this, updateInterval.toLong())
            }
        })
    }

    fun navigateTrajectory(waypoints: List<Triple<Double, Double, Double>>, finalYaw: Double) {
        if (waypoints.isEmpty()) {
            // No waypoints we do nothing.
            return
        }

        val updateIntervalMs = 100.0
        val maxSpeed = 10.0
        val maxYawRate = 30.0

        // PID identical to navigateToWaypointWithPID, used only for the final waypoint
        val distancePID = PID(kp = 0.65, ki = 0.0001, kd = 0.001, dt = updateIntervalMs / 1000.0, outputLimits = 0.0 to maxSpeed)
        val yawPID = PID(kp = 3.0, ki = 0.0, kd = 0.0, dt = updateIntervalMs / 1000.0, outputLimits = -maxYawRate to maxYawRate)

        virtualStickVM.enableVirtualStickAdvancedMode()
        isWaypointReached = false
        isIntermediaryWaypointReached = false

        var currentIndex = 0
        val controlLoop = Handler(Looper.getMainLooper())

        controlLoop.post(object : Runnable {
            override fun run() {
                val currentPosition = getLocation3D()
                val currentYaw = getHeading()
                // Current target waypoint
                val targetWp = waypoints[currentIndex]
                val targetLatitude = targetWp.first
                val targetLongitude = targetWp.second
                val targetAltitude = targetWp.third

                // Distance to the current waypoint
                val distance = calculateDistance(
                        targetLatitude,
                        targetLongitude,
                        currentPosition.latitude,
                        currentPosition.longitude
                )

                // Compute yawError based on final Yaw
                val yawError = normalizeAngle(finalYaw - currentYaw)
                val angularVelocity = yawPID.update(yawError)

                // Movement direction
                val movementDirection = calculateBearing(
                        currentPosition.latitude,
                        currentPosition.longitude,
                        targetLatitude,
                        targetLongitude
                ).toDouble()

                // Are we at the last waypoint?
                val isLastWaypoint = (currentIndex == waypoints.size - 1)

                // Speed deduction: PID if we're moving towards the last waypoint.
                val targetSpeed = if (isLastWaypoint) {
                    // PID control for the last waypoint, we want the movement to be smooth.
                    distancePID.update(distance)
                } else {
                    // Intermediary waypoint: we use a constant speed.
                    maxSpeed
                }

                // Direction of the Drone.
                val movementDirectionRelative = normalizeAngle(movementDirection - currentYaw)
                val pitch = targetSpeed * cos(Math.toRadians(movementDirectionRelative))
                val roll = targetSpeed * sin(Math.toRadians(movementDirectionRelative))

                // Altitude
                val altError = targetAltitude - currentPosition.altitude

                val distanceThresholdLast = 1
                val distanceThresholdInter = 2.0
                val yawThreshold = 4
                val altitudeThreshold = 1.5

                val waypointReached = if (isLastWaypoint) {
                    // Strict criterias if it's the last waypoint.
                    (distance < distanceThresholdLast && abs(yawError) < yawThreshold && abs(altError) < altitudeThreshold)
                } else {
                    // Simplified criteria for the intermediary waypoints.
                    (distance < distanceThresholdInter)
                }

                if (waypointReached) {
                    isIntermediaryWaypointReached = true
                    if (!isLastWaypoint) {
                        // We target the next waypoint.
                        currentIndex++
                    } else {
                        // Last waypoint
                        setStick(0F, 0F, 0F, 0F)
                        isWaypointReached = true
                        return
                    }
                } else {
                    isIntermediaryWaypointReached = false
                }


                val flightControlParam = VirtualStickFlightControlParam().apply {
                    // DJI SDK is cursed, so we do inverse roll and pitch again.
                    this.pitch = roll
                    this.roll = pitch
                    this.yaw = angularVelocity
                    this.verticalThrottle = targetAltitude
                    this.verticalControlMode = VerticalControlMode.POSITION
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)

                controlLoop.postDelayed(this, updateIntervalMs.toLong())
            }
        })
    }

    // Getter pour isWaypointReached
    fun isWaypointReached(): Boolean {
        return isWaypointReached
    }

    // Getter pour isYawReached
    fun isYawReached(): Boolean {
        return isYawReached
    }

    // Idem pour isAltitudeReached, etc.
    fun isAltitudeReached(): Boolean {
        return isAltitudeReached
    }

    fun isIntermediaryWaypointReached(): Boolean {
        return isIntermediaryWaypointReached
    }


}
