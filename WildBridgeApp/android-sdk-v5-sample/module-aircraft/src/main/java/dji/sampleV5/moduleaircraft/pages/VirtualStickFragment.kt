package dji.sampleV5.moduleaircraft.pages

import android.os.Bundle
import android.view.*
import androidx.fragment.app.activityViewModels
import androidx.fragment.app.viewModels
import dji.sampleV5.moduleaircraft.R
import dji.sampleV5.moduleaircraft.models.BasicAircraftControlVM
import dji.sampleV5.moduleaircraft.models.VirtualStickVM
import dji.sampleV5.modulecommon.keyvalue.KeyValueDialogUtil
import dji.sampleV5.modulecommon.models.LiveStreamVM
import dji.sampleV5.modulecommon.pages.DJIFragment
import dji.sampleV5.modulecommon.util.Helper
import dji.sampleV5.modulecommon.util.ToastUtils
import dji.sdk.keyvalue.value.common.*
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.video.decoder.DecoderOutputMode
import dji.v5.common.video.decoder.DecoderState
import dji.v5.common.video.decoder.VideoDecoder
import dji.v5.common.video.interfaces.IVideoDecoder
import dji.v5.manager.aircraft.virtualstick.Stick
import dji.v5.utils.common.JsonUtil
import dji.v5.utils.common.StringUtils
import dji.v5.ux.core.extension.disableHardwareAccelerated
import kotlinx.android.synthetic.main.frag_virtual_stick_page.*
import kotlin.math.abs
import android.os.Handler
import android.os.Looper
import dji.sdk.keyvalue.key.*
import dji.sdk.keyvalue.value.camera.CameraVideoStreamSourceType
import dji.v5.et.*




/**
 * This class implements the functionality useful for WildBridge, it was adapted from the Virtual
 * stick fragment as well as the livestream fragment.
 */

class VirtualStickFragment : DJIFragment(), SurfaceHolder.Callback {

    // AIRCRAFT STATE AND CONTROL
    private val basicAircraftControlVM: BasicAircraftControlVM by activityViewModels()
    private val virtualStickVM: VirtualStickVM by activityViewModels()

    // LIVESTREAM
    private val liveStreamVM: LiveStreamVM by viewModels()
    private var videoDecoder: IVideoDecoder? = null
    private lateinit var surfaceView: SurfaceView

    private val streamUsername: String =
            "aaa" // Stream is accessible at "rtsp://<streamUsername>:<streamPassword>@<device IPV4>:<streamPort>/streaming/live/1"
    private val streamPassword: String = "aaa"
    private val streamPort: Int = 8554
<<<<<<< HEAD

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
    private var isYawAdjusted = false
    private var isWaypointReachedBeforeYaw = false

    // STREAM STABILITY
    private var isFirstConnectionSuccessful = false
    private var previousBitrate: Int? = 0
=======
    private val videoSourceKey: DJIKey<CameraVideoStreamSourceType> =
            CameraKey.KeyCameraVideoStreamSource.create()
>>>>>>> WildBridgeV2

    // VIEW
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(R.layout.frag_virtual_stick_page, container, false)
    }

    private fun initView(view: View) {
        surfaceView = view.findViewById(R.id.live_stream_surface_view2)
        surfaceView.holder.addCallback(this)
    }

<<<<<<< HEAD
    private fun enableVirtualStick() {
        virtualStickVM.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("enableVirtualStick success.")
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("enableVirtualStick error,$error")
            }
        })
    }

    private fun disableVirtualStick() {
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

    private fun gotoYaw(targetYaw: Double) {
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

    private fun gotoAltitude(targetAltitude: Double) {
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

    private fun gotoWP(targetLatitude: Double, targetLongitude: Double, targetAltitude: Double, targetYaw: Double) {
        val controlLoop = Handler(Looper.getMainLooper())
        val updateInterval: Long = 100 // Update every 100 ms

        // Enable virtual stick with function defined before
        isWaypointReached = false
        isWaypointReachedBeforeYaw = false
        isYawAdjusted = false

        virtualStickVM.enableVirtualStickAdvancedMode()

        controlLoop.post(object : Runnable {
            override fun run() {

                if (isWaypointReached) return //

                val currentPosition = getLocation3D()
                val distanceToWaypoint = calculateDistance(
                    targetLatitude,
                    targetLongitude,
                    currentPosition.latitude,
                    currentPosition.longitude
                )

                val altError = targetAltitude - currentPosition.altitude

                if (!isWaypointReachedBeforeYaw){

                    if (distanceToWaypoint < 0.5 && abs(altError) < 0.5) { // Stop if close enough to the waypoint
                        setStick(0F, 0F, 0F, 0F)
                        isWaypointReachedBeforeYaw = true
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

                }

                else{
                    // Second phase: Adjust yaw towards targetYaw
                    val currentYaw = getHeading()
                    var yawError = targetYaw - currentYaw
                    yawError = normalizeAngle(yawError)

                    if (abs(yawError) < 4) { // Yaw aligned
                        isYawAdjusted = true
                        isWaypointReached = true
                        setStick(0F, 0F, 0F, 0F)
                        return // Stop control loop
                    }

                    val flightControlParam = VirtualStickFlightControlParam().apply {
                        this.pitch = 0.0
                        this.roll = 0.0
                        this.yaw = targetYaw
                        this.verticalThrottle = targetAltitude
                        this.verticalControlMode = VerticalControlMode.POSITION
                        this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                        this.yawControlMode = YawControlMode.ANGLE
                        this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                    }

                    virtualStickVM.sendVirtualStickAdvancedParam(flightControlParam)
                }
                // Schedule the next update
                controlLoop.postDelayed(this, updateInterval)
            }
        })
    }

    private fun navigateToWaypointWithPIDtuning(targetLatitude: Double, targetLongitude: Double, targetAlt: Double, targetYaw: Double, kp_position: Double, ki_position: Double, kd_position: Double, kp_yaw:Double, ki_yaw:Double, kd_yaw:Double) {

        val updateInterval = 100.0  // Update every 100 ms
        val maxSpeed = 5.0 // meters per second
        val maxYawRate = 30.0 // degrees per second

        virtualStickVM.enableVirtualStickAdvancedMode()

        val distancePID = PID(kp_position, ki_position, kd_position, updateInterval/1000, 0.0 to maxSpeed)
        val yawPID = PID(kp_yaw, ki_yaw, kd_yaw, updateInterval/1000, -maxYawRate to maxYawRate)

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

                if (distance < 0.5 && abs(yawError) < 1.5 && abs(altError) < 0.5) { // Stop if close enough to the waypoint
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

    // Helper function to calculate the bearing between two coordinates
    // Azimuth in French, which direction we should go
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

    // Helper function to normalize an angle to the range [-180, 180]
    fun normalizeAngle(angle: Double): Double {
        var adjustedAngle = angle % 360
        if (adjustedAngle > 180) adjustedAngle -= 360
        if (adjustedAngle < -180) adjustedAngle += 360
        return adjustedAngle
    }
=======
    private var isFirstConnectionSuccessful = false
    private var previousBitrate: Int? = 0
>>>>>>> WildBridgeV2

    // Loop
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        widget_horizontal_situation_indicator.setSimpleModeEnable(false)
        view.disableHardwareAccelerated()

        // View init
        initView(view)
        initBtnClickListener()

        // Functionality init
        startStream()
<<<<<<< HEAD
        // observeStreamBitrate() Needed if you're using an old android device
    }

    // SERVER
    private fun startControlServer() {
        // Creating parameter getter
        val flightSpeed: DJIKey<Velocity3D> = FlightControllerKey.KeyAircraftVelocity.create()
        fun getSpeed(): Velocity3D = flightSpeed.get(Velocity3D(0.0, 0.0, 0.0))

        val compassHeadKey: DJIKey<Double> = FlightControllerKey.KeyCompassHeading.create()
        fun getHeading(): Double = (compassHeadKey.get(0.0)).toDouble()

        val attitudeKey: DJIKey<Attitude> = FlightControllerKey.KeyAircraftAttitude.create()
        fun getAttitude(): Attitude = attitudeKey.get(Attitude(0.0, 0.0, 0.0))

        val satelliteCountKey: DJIKey<Int> = FlightControllerKey.KeyGPSSatelliteCount.create()
        fun getSatelliteCount(): Int = satelliteCountKey.get(-1)

        val location3DKey: DJIKey<LocationCoordinate3D> = FlightControllerKey.KeyAircraftLocation3D.create()
        fun getLocation3D(): LocationCoordinate3D = location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))

        val homeLocationKey: DJIKey<LocationCoordinate2D> = FlightControllerKey.KeyHomeLocation.create()
        fun getLocationHome(): LocationCoordinate2D = homeLocationKey.get(LocationCoordinate2D())

        val gimbalAttitudeKey: DJIKey<Attitude> = GimbalKey.KeyGimbalAttitude.create()
        fun getGimbalAttitudeKey(): Attitude = gimbalAttitudeKey.get(Attitude(0.0, 0.0, 0.0))

        val gimbalJointAttitudeKey: DJIKey<Attitude> = GimbalKey.KeyGimbalJointAttitude.create()
        fun getJointAttitude(): Attitude = gimbalJointAttitudeKey.get(Attitude(0.0, 0.0, 0.0))

        val cameraZoomFocalLengthKey: DJIKey<Int> = CameraKey.KeyCameraZoomFocalLength.create()
        fun getCameraZoomFocalLength(): Int = cameraZoomFocalLengthKey.get(-1)

        val cameraOpticalFocalLengthKey: DJIKey<Int> =
            CameraKey.KeyCameraOpticalZoomFocalLength.create()

        fun getCameraOpticalFocalLength(): Int = cameraOpticalFocalLengthKey.get(-1)

        val cameraHybridFocalLengthKey: DJIKey<Int> =
            CameraKey.KeyCameraHybridZoomFocalLength.create()

        fun getCameraHybridFocalLength(): Int = cameraHybridFocalLengthKey.get(-1)

        val cameraApertureKey: DJIKey<CameraAperture> = CameraKey.KeyAperture.create()
        fun getCameraAperture(): CameraAperture = cameraApertureKey.get(CameraAperture.UNKNOWN)

        // Starting Jetty server and defining nodes
        embeddedServer(Jetty, port = 8080) {
            routing {

                post("/send/takeoff") {
                    basicAircraftControlVM.startTakeOff(object :
                        CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                        override fun onSuccess(t: EmptyMsg?) {
                            ToastUtils.showToast("start takeOff onSuccess.")
                        }

                        override fun onFailure(error: IDJIError) {
                            ToastUtils.showToast("start takeOff onFailure,$error")
                        }
                    })
                }

                post("/send/land") {
                    basicAircraftControlVM.startLanding(object :
                        CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                        override fun onSuccess(t: EmptyMsg?) {
                            ToastUtils.showToast("start landing onSuccess.")
                        }

                        override fun onFailure(error: IDJIError) {
                            ToastUtils.showToast("start landing onFailure,$error")
                        }
                    })
                }

                post("/send/RTH") {
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

                post("/send/stick") {
                    val text = call.receiveText()
                    val cmd = text.split(",")
                    val lx = cmd[0].toFloat()
                    val ly = cmd[1].toFloat()
                    val rx = cmd[2].toFloat()
                    val ry = cmd[3].toFloat()
                    setStick(lx, ly, rx, ry)
                    call.respondText("Received: leftX: $lx, leftY: $ly, rightX: $rx, rightY: $ry")
                }

                post("/send/gimbal/pitch") {
                    val text = call.receiveText()
                    val cmd = text.split(",")
                    val roll = cmd[0].toDouble()
                    val pitch = cmd[1].toDouble()
                    val yaw = cmd[2].toDouble()
                    // {'mode':0,'pitch':0,'roll':0,'yaw':0,'pitchIgnored':false,'rollIgnored':false,'yawIgnored':false,'duration':0,'jointReferenceUsed':false,'timeout':0}
                    val rot = GimbalAngleRotation(
                        GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                        pitch,
                        roll,
                        yaw,
                        false,
                        true,
                        true,
                        0.1,
                        false,
                        0
                    )
                    gimbalKey.action(rot)
                    call.respondText("Received: roll: $roll, pitch: $pitch, yaw: $yaw")
                }

                post("/send/gimbal/yaw") {
                    val text = call.receiveText()
                    val cmd = text.split(",")
                    val roll = cmd[0].toDouble()
                    val pitch = cmd[1].toDouble()
                    val yaw = cmd[2].toDouble()
                    // {'mode':0,'pitch':0,'roll':0,'yaw':0,'pitchIgnored':false,'rollIgnored':false,'yawIgnored':false,'duration':0,'jointReferenceUsed':false,'timeout':0}
                    val rot = GimbalAngleRotation(
                            GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                            pitch,
                            roll,
                            yaw,
                            true,
                            true,
                            false,
                            0.1,
                            false,
                            0
                    )
                    gimbalKey.action(rot)
                    call.respondText("Received: roll: $roll, pitch: $pitch, yaw: $yaw")
                }

                post("/send/gotoYaw") {
                    val text = call.receiveText()
                    val cmd = text.split(",")
                    val yaw = cmd[0].toDouble()
                    gotoYaw(yaw)
                    call.respondText("Received: yaw: $yaw")
                }

                post("/send/gotoAltitude") {
                    val text = call.receiveText()
                    val cmd = text.split(",")
                    val targetAltitude = cmd[0].toDouble()
                    gotoAltitude(targetAltitude)
                    call.respondText("Received: Altitude: $targetAltitude")
                }

                post("/send/camera/zoom") {
                    val targetZoom = call.receiveText().toDouble()
                    val zoomBefore = zoomKey.get()
                    zoomKey.set(targetZoom)
                    val zoomAfter = zoomKey.get()
                    call.respondText("Received: zoom: $targetZoom")
                }

                post("/send/abortMission") {
                    setStick(0.0f, 0.0f, 0.0f, 0.0f)
                    disableVirtualStick()
                    call.respondText("Received: abortMission")
                }

                post("/send/enableVirtualStick") {
                    enableVirtualStick()
                    call.respondText("Received: enableVirtualStick")
                }

                post("/send/camera/startRecording") {
                    startRecording.action()
                    call.respondText("Received: camera start recording")
                }

                post("/send/camera/stopRecording") {
                    stopRecording.action()
                    call.respondText("Received: camera stop recording")
                }

                get("/status/camera/isRecording") {
                    call.respondText(isRecording.get().toString())
                }

                post("/send/gotoWP") {
                    val text = call.receiveText()
                    val cmd = text.split(",")

                    if (cmd.size < 3) {
                        call.respondText("Invalid input. Expected format: lat,lon,alt")
                        return@post
                    }

                    try {
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()

                        // Call gotoWP with the parsed coordinates
                        //gotoWP(latitude, longitude, altitude)

                        call.respondText("Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude")
                    } catch (e: Exception) {
                        call.respondText("Error processing waypoint command: ${e.message}")
                    }
                }

                post("/send/gotoWPwithPID") {
                    val text = call.receiveText()
                    val cmd = text.split(",")

                    if (cmd.size < 4) {
                        call.respondText("Invalid input. Expected format: lat,lon,alt,yaw")
                        return@post
                    }

                    try {
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()
                        val yaw = cmd[3].toDouble()

                        // Call navigateToWaypointWithPID with the parsed coordinates and yaw
                        gotoWP(latitude, longitude, altitude, yaw)

                        call.respondText("Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude, Yaw=$yaw")
                    } catch (e: Exception) {
                        call.respondText("Error processing waypoint command: ${e.message}")
                    }
                }

                post("/send/gotoWPwithPIDtuning") {
                    val text = call.receiveText()
                    val cmd = text.split(",")

                    if (cmd.size < 10) {
                        call.respondText("Invalid input. Expected format: lat,lon,alt,yaw")
                        return@post
                    }

                    try {
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()
                        val yaw = cmd[3].toDouble()
                        val kp_position = cmd[4].toDouble()
                        val ki_position = cmd[5].toDouble()
                        val kd_position = cmd[6].toDouble()
                        val kp_yaw = cmd[7].toDouble()
                        val ki_yaw = cmd[8].toDouble()
                        val kd_yaw = cmd[9].toDouble()

                        // Call navigateToWaypointWithPID with the parsed coordinates and yaw
                        navigateToWaypointWithPIDtuning(latitude, longitude, altitude, yaw, kp_position, ki_position, kd_position, kp_yaw, ki_yaw, kd_yaw)

                        call.respondText("Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude, Yaw=$yaw")
                    } catch (e: Exception) {
                        call.respondText("Error processing waypoint command: ${e.message}")
                    }
                }

                get("/aircraft/rcStickValues") {
                    val rcStickValues = virtual_stick_info_tv.text
                    call.respondText(rcStickValues.toString())
                }

                get("/") {
                    call.respondText("Hello!\nYou are connected to WildBridge.")
                }

                get("/aircraft/allStates") {
                    val speed = getSpeed().toString()
                    val heading = getHeading().toString()
                    val attitude = getAttitude().toString()
                    val gimbalJointAttitude = getJointAttitude().toString()
                    val gimbalAttitude = getGimbalAttitudeKey().toString()
                    val location = getLocation3D().toString()
                    val zoomFl = getCameraZoomFocalLength().toString()
                    val hybridFl = getCameraHybridFocalLength().toString()
                    val opticalFl = getCameraOpticalFocalLength().toString()
                    val zoomRatio = zoomKey.get().toString()
                    val batteryLevel = getBatteryLevel().toString()
                    val satelliteCount = getSatelliteCount().toString()

                    val builder = StringBuilder()
                    builder.append("{\"speed\":")
                    builder.append(speed)
                    builder.append(",\"heading\":")
                    builder.append(heading)
                    builder.append(",\"attitude\":")
                    builder.append(attitude)
                    builder.append(",\"location\":")
                    builder.append(location)
                    builder.append(",\"gimbalAttitude\":")
                    builder.append(gimbalAttitude)
                    builder.append(",\"gimbalJointAttitude\":")
                    builder.append(gimbalJointAttitude)
                    builder.append(",\"zoomFl\":")
                    builder.append(zoomFl)
                    builder.append(",\"hybridFl\":")
                    builder.append(hybridFl)
                    builder.append(",\"opticalFl\":")
                    builder.append(opticalFl)
                    builder.append(",\"zoomRatio\":")
                    builder.append(zoomRatio)
                    builder.append(",\"batteryLevel\":")
                    builder.append(batteryLevel)
                    builder.append(",\"satelliteCount\":")
                    builder.append(satelliteCount)
                    builder.append("}")

                    call.respondText(builder.toString())
                }
                get("/aircraft/speed") {
                    call.respondText(getSpeed().toString())
                }

                get("/home/location") {
                    call.respondText(getLocationHome().toString())
                }

                get("/aircraft/heading") {
                    call.respondText(getHeading().toString())
                }
                get("/aircraft/attitude") {
                    call.respondText(getAttitude().toString())
                }
                get("/aircraft/location") {
                    call.respondText(getLocation3D().toString())
                }
                get("/aircraft/gimbalAttitude") {
                    call.respondText(getGimbalAttitudeKey().toString())
                }
                get("/status/waypointReached") {
                    val status = if (isWaypointReached) "true" else "false"
                    call.respondText(status)
                }
                get("/status/yawReached") {
                    val status = if (isYawReached) "true" else "false"
                    call.respondText(status)
                }

                get("/status/altitudeReached") {
                    val status = if (isAltitudeReached) "true" else "false"
                    call.respondText(status)
                }

            }
        }.start(wait = false)
    }

    private fun setStick(
        leftX: Float = 0F,
        leftY: Float = 0F,
        rightX: Float = 0F,
        rightY: Float = 0F
    ) {
        watchDog = watchDogReset
        virtualStickVM.setLeftPosition(
            (leftX * Stick.MAX_STICK_POSITION_ABS).toInt(),
            (leftY * Stick.MAX_STICK_POSITION_ABS).toInt()
        )
        virtualStickVM.setRightPosition(
            (rightX * Stick.MAX_STICK_POSITION_ABS).toInt(),
            (rightY * Stick.MAX_STICK_POSITION_ABS).toInt()
        )
    }

    private fun startWatchDog() {
        val mainHandler = Handler(Looper.getMainLooper())
        mainHandler.post(object : Runnable {
            override fun run() {
                watchDog--
                if (watchDog <= 0) {
                    watchDog = watchDogReset
                    setStick(0F, 0F, 0F, 0F)
                }
                updateVirtualStickInfo()
                mainHandler.postDelayed(this, watchDogPeriod)
            }
        })
=======
        observeStreamBitrate() //Needed if you're using an old android device
>>>>>>> WildBridgeV2
    }

    // BUTTONS AND JOYSTICKS
    private fun initBtnClickListener() {
        btn_enable_virtual_stick.setOnClickListener {
          
            virtualStickVM.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("enableVirtualStick success.")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("enableVirtualStick error,$error")
                }
            })
        }
        btn_disable_virtual_stick.setOnClickListener {
            virtualStickVM.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("disableVirtualStick success.")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("disableVirtualStick error,${error})")
                }
            })
        }
        btn_set_virtual_stick_speed_level.setOnClickListener {
            val speedLevels = doubleArrayOf(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
            initPopupNumberPicker(Helper.makeList(speedLevels)) {
                virtualStickVM.setSpeedLevel(speedLevels[indexChosen[0]])
                resetIndex()
            }
        }
        btn_take_off.setOnClickListener {
            basicAircraftControlVM.startTakeOff(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    ToastUtils.showToast("start takeOff onSuccess.")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("start takeOff onFailure,$error")
                }
            })
        }

        btn_landing.setOnClickListener {
            basicAircraftControlVM.startLanding(object :
                CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
                override fun onSuccess(t: EmptyMsg?) {
                    ToastUtils.showToast("start landing onSuccess.")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("start landing onFailure,$error")
                }
            })
        }
        btn_use_rc_stick.setOnClickListener {
            virtualStickVM.useRcStick.value = virtualStickVM.useRcStick.value != true
            if (virtualStickVM.useRcStick.value == true) {
                ToastUtils.showToast(
                    "After it is turned on," +
                            "the joystick value of the RC will be used as the left/ right stick value"
                )
            }
        }
        btn_set_virtual_stick_advanced_param.setOnClickListener {
            KeyValueDialogUtil.showInputDialog(
                activity, "Set Virtual Stick Advanced Param",
                JsonUtil.toJson(virtualStickVM.virtualStickAdvancedParam.value), "", false
            ) {
                it?.apply {
                    val param = JsonUtil.toBean(this, VirtualStickFlightControlParam::class.java)
                    if (param == null) {
                        ToastUtils.showToast("Value Parse Error")
                        return@showInputDialog
                    }
                    virtualStickVM.virtualStickAdvancedParam.postValue(param)
                }
            }
        }
        btn_send_virtual_stick_advanced_param.setOnClickListener {
            virtualStickVM.virtualStickAdvancedParam.value?.let {
                virtualStickVM.sendVirtualStickAdvancedParam(it)
            }
        }
        btn_enable_virtual_stick_advanced_mode.setOnClickListener {
            virtualStickVM.enableVirtualStickAdvancedMode()
        }
        btn_disable_virtual_stick_advanced_mode.setOnClickListener {
            virtualStickVM.disableVirtualStickAdvancedMode()
        }
    }

    // STREAMING FUNCTIONS

    private fun stopStream() {
        liveStreamVM.stopStream(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast(StringUtils.getResStr(dji.sampleV5.modulecommon.R.string.msg_stop_live_stream_success))
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast(
                    StringUtils.getResStr(
                        dji.sampleV5.modulecommon.R.string.msg_stop_live_stream_failed,
                        error.description()
                    )
                )
            }
        })
    }

    private fun startStream() {
        if (streamUsername.isEmpty() || streamPassword.isEmpty() || streamPort <= 0) {
            ToastUtils.showToast("Invalid Stream Configurations")
            return
        }

        // Video Source
        videoSourceKey.set(CameraVideoStreamSourceType.ZOOM_CAMERA)

        // RTSP Config
        liveStreamVM.setRTSPConfig(streamUsername, streamPassword, streamPort)

        // Start the stream
        liveStreamVM.startStream(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                isFirstConnectionSuccessful = true
                ToastUtils.showToast("Streaming successfully started")
            }

            override fun onFailure(error: IDJIError) {
                isFirstConnectionSuccessful = false
                ToastUtils.showToast("Fail starting streaming : ${error.description()}")
                scheduleReconnect()
            }
        })
    }

    private fun scheduleReconnect() {
        Handler(Looper.getMainLooper()).postDelayed({
            if (!isFirstConnectionSuccessful) {
                ToastUtils.showToast("We attempt to reconnect...")
                startStream() // We try to reconnect
            }
        }, 2000)
    }

    private fun observeStreamBitrate() {
        liveStreamVM.curLiveStreanmStatus.observe(viewLifecycleOwner) { streamStatus ->
            streamStatus?.let {
                val currentBitrate = it.vbps

                // Check if the previous bitrate was positive and the current bitrate is zero
                if (isFirstConnectionSuccessful && (previousBitrate ?: -1) > 0 && currentBitrate == 0) {
                    ToastUtils.showToast("Bitrate dropped to zero, restarting stream...")
                    restartStream()
                }

                // Update the previous bitrate
                previousBitrate = currentBitrate
            }
        }
    }

    override fun surfaceCreated(holder: SurfaceHolder) {
        if (videoDecoder == null) {
            videoDecoder = VideoDecoder(
                this@VirtualStickFragment.context,
                liveStreamVM.getVideoChannel(),
                DecoderOutputMode.SURFACE_MODE,
                surfaceView.holder,
                live_stream_surface_view2.width,
                live_stream_surface_view2.height,
                true
            )
        } else if (videoDecoder?.decoderStatus == DecoderState.PAUSED) {
            videoDecoder?.onResume()
        }
    }

    private fun restartStream() {
        stopStream()
        Handler(Looper.getMainLooper()).postDelayed({
            startStream()
        }, 100) // 2 seconds break before restarting
    }

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, wiupdateIntervalh: Int, height: Int) {
        surfaceCreated(holder)
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        videoDecoder?.let {
            videoDecoder?.onPause()
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        videoDecoder?.let {
            it.destroy()
        }
        stopStream()
    }
}