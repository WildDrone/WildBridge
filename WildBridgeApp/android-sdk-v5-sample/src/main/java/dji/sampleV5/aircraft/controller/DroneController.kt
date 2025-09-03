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



object DroneController {

    private lateinit var basicAircraftControlVM: BasicAircraftControlVM
    lateinit var virtualStickVM: VirtualStickVM

    fun init(basicVM: BasicAircraftControlVM, stickVM: VirtualStickVM ) {
        basicAircraftControlVM = basicVM
        virtualStickVM = stickVM
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

    fun navigateTrajectory(
        waypoints: List<Triple<Double, Double, Double>>,
        lookaheadDistance: Double = 5.5,
        cruiseSpeed: Double = 5.0,
        minSpeedFinal: Double = 1.0,
        slowdownRadius: Double = 4.0
    ) {
        if (waypoints.size < 2) return

        val updateIntervalMs = 100L

        var currentIndex = 0
        isWaypointReached = false
        isIntermediaryWaypointReached = false

        virtualStickVM.enableVirtualStickAdvancedMode()
        val controlLoop = Handler(Looper.getMainLooper())

        // Helper: Compute great-circle distance (meters) between two lat/lon
        fun calculateDistance(latA: Double, lonA: Double, latB: Double, lonB: Double): Double {
            val earthR = 6371000.0
            val phi1 = Math.toRadians(latA)
            val phi2 = Math.toRadians(latB)
            val deltaPhi = Math.toRadians(latB - latA)
            val deltaLambda = Math.toRadians(lonB - lonA)
            val a = Math.sin(deltaPhi/2) * Math.sin(deltaPhi/2) +
                    Math.cos(phi1) * Math.cos(phi2) *
                    Math.sin(deltaLambda/2) * Math.sin(deltaLambda/2)
            val c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
            return earthR * c
        }

        // Helper: Compute bearing from (lat1, lon1) to (lat2, lon2)
        fun calculateBearing(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
            val phi1 = Math.toRadians(lat1)
            val phi2 = Math.toRadians(lat2)
            val deltaLambda = Math.toRadians(lon2 - lon1)
            val y = Math.sin(deltaLambda) * Math.cos(phi2)
            val x = Math.cos(phi1) * Math.sin(phi2) -
                    Math.sin(phi1) * Math.cos(phi2) * Math.cos(deltaLambda)
            val bearing = Math.toDegrees(Math.atan2(y, x))
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

        controlLoop.post(object : Runnable {
            override fun run() {
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
                val projLat = start.first + (end.first - start.first) * projRatio
                val projLon = start.second + (end.second - start.second) * projRatio
                val projAlt = start.third + (end.third - start.third) * projRatio

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

                val pitch = targetSpeed * Math.cos(Math.toRadians(moveDirRel))
                val roll = targetSpeed * Math.sin(Math.toRadians(moveDirRel))

                // Stop criteria: last segment, close to endpoint, and altitude close
                val reached = isLastSegment &&
                        (calculateDistance(current.latitude, current.longitude, end.first, end.second) < 0.8) &&
                        (Math.abs(targetAlt - current.altitude) < 1.0)

                if (reached) {
                    setStick(0F, 0F, 0F, 0F)
                    isWaypointReached = true
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

                virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)
                controlLoop.postDelayed(this, updateIntervalMs)
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
