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
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.zip.ZipEntry
import java.util.zip.ZipFile

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

    // === DJI Native Wayline (KMZ) flow ===
    private fun generateTrajectoryName(): String {
        val dateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault())
        return "trajectory_${dateFormat.format(Date())}"
    }

    private fun createWaypointFromLatLon(
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

        val gimbalParam = WaylineWaypointGimbalHeadingParam().apply {
            headingMode = WaylineWaypointGimbalHeadingMode.find(0)
            pitchAngle = 30.0
        }
        waypoint.gimbalHeadingParam = gimbalParam
        waypoint.isWaylineWaypointGimbalHeadingParamSet = true
        waypoint.useGlobalGimbalHeadingParam = false

        waypointInfo.waylineWaypoint = waypoint
        waypointInfo.actionInfos = ArrayList()
        return waypointInfo
    }

    private fun createWaylineMission(): WaylineMission {
        val m = WaylineMission()
        val now = System.currentTimeMillis().toDouble()
        m.createTime = now
        m.updateTime = now
        return m
    }

    private fun createMissionConfig(): WaylineMissionConfig {
        val c = WaylineMissionConfig()
        c.flyToWaylineMode = WaylineFlyToWaylineMode.SAFELY
        // Use KMZ's settings; we set defaults commonly used
        c.finishAction = WaylineFinishedAction.NO_ACTION
        c.droneInfo = WaylineDroneInfo()
        c.securityTakeOffHeight = 20.0
        c.isSecurityTakeOffHeightSet = true
        c.exitOnRCLostBehavior = WaylineExitOnRCLostBehavior.EXCUTE_RC_LOST_ACTION
        c.exitOnRCLostType = WaylineExitOnRCLostAction.GO_BACK
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
        info.actionGroups = ArrayList()
        info.globalFlightHeight = 100.0
        info.isGlobalFlightHeightSet = true
        info.globalTurnMode = WaylineWaypointTurnMode.TO_POINT_AND_PASS_WITH_CONTINUITY_CURVATURE
        info.useStraightLine = false
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
        info.pitchMode = WaylineWaypointPitchMode.USE_POINT_SETTING
        return info
    }

    private fun createTemplate(waypointInfoModels: List<WaypointInfoModel>): Template {
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
        t.autoFlightSpeed = 12.0
        t.payloadParam = ArrayList()
        return t
    }

    private fun extractWaylineIdsFromKmz(kmzPath: String): ArrayList<Int> {
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

    fun navigateTrajectoryNative(userWaypoints: List<Triple<Double, Double, Double>>) {
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
        val template = createTemplate(wpModels)

        // Generate KMZ
        val missionName = generateTrajectoryName()
        val kmzOutPath = kmzDir + missionName + ".kmz"
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
