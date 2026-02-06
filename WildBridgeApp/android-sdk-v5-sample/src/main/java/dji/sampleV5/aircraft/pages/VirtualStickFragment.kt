package dji.sampleV5.aircraft.pages
import android.annotation.SuppressLint
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.Surface
import android.view.SurfaceHolder
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.databinding.FragVirtualStickPageBinding
import dji.sampleV5.aircraft.keyvalue.KeyValueDialogUtil
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.models.SimulatorVM
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.models.LiveStreamVM
import dji.sampleV5.aircraft.util.Helper
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.GimbalKey
import dji.sdk.keyvalue.value.common.Attitude
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.sdk.keyvalue.value.common.Velocity3D
import dji.sdk.keyvalue.value.flightcontroller.VirtualStickFlightControlParam
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotation
import dji.sdk.keyvalue.value.gimbal.GimbalAngleRotationMode
import dji.sdk.keyvalue.value.flightcontroller.LowBatteryRTHInfo
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.set
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.datacenter.livestream.StreamQuality
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.utils.common.JsonUtil
import java.io.IOException
import java.net.Inet4Address
import java.net.NetworkInterface
import java.net.ServerSocket
import java.util.Collections
import dji.v5.ux.core.util.DataProcessor
import dji.sdk.keyvalue.key.KeyTools
import dji.sampleV5.aircraft.controller.DroneController
import dji.sampleV5.aircraft.server.TelemetryServer

// Import for custom HTTP server implementation
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.io.PrintWriter
import java.net.Socket
import java.util.concurrent.Executors
import kotlin.concurrent.thread
import dji.v5.manager.KeyManager

import dji.sdk.keyvalue.value.flightcontroller.FlightMode
import dji.sampleV5.aircraft.webrtc.WebRTCStreamer
import dji.sampleV5.aircraft.webrtc.WebRTCMediaOptions

/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/5/11
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */

class VirtualStickFragment : DJIFragment() {

    companion object {
        private const val TAG = "VirtualStickFragment"
    }

    private val basicAircraftControlVM: BasicAircraftControlVM by activityViewModels()
    private val virtualStickVM: VirtualStickVM by activityViewModels()
    private val simulatorVM: SimulatorVM by activityViewModels()
    private val liveStreamVM: LiveStreamVM by activityViewModels()
    private var binding: FragVirtualStickPageBinding? = null

    // Camera stream related variables
    private val cameraStreamManager = MediaDataCenter.getInstance().cameraStreamManager
    private var cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN
    private var cameraStreamSurface: Surface? = null
    private var cameraStreamWidth = -1
    private var cameraStreamHeight = -1
    private var cameraStreamScaleType: ICameraStreamManager.ScaleType = ICameraStreamManager.ScaleType.CENTER_INSIDE
    private var isVideoFeedEnabled: Boolean = true

    // Simple HTTP Server instance
    private var httpServer: SimpleHttpServer? = null
    private var telemetryServer: TelemetryServer? = null
    private var isHomePointSetLatch = false

    // WebRTC streaming
    private var webRTCStreamer: WebRTCStreamer? = null
    private var isWebRTCMode = true  // Start with WebRTC by default
    private val wEBRTCPORT = 8082  // Use different port than telemetry server

    // --- Remaining flight time style data (similar to RemainingFlightTimeWidgetModel) ---
    private val chargeRemainingProcessor: DataProcessor<Int> = DataProcessor.create(0)
    private val goHomeAssessmentProcessor: DataProcessor<LowBatteryRTHInfo> = DataProcessor.create(LowBatteryRTHInfo())
    private val seriousLowBatteryThresholdProcessor: DataProcessor<Int> = DataProcessor.create(0)
    private val lowBatteryThresholdProcessor: DataProcessor<Int> = DataProcessor.create(0)
    private val timeNeededToLandProcessor: DataProcessor<Int> = DataProcessor.create(0)

    private val chargeRemainingKey = KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent)
    private val goHomeAssessmentKey = KeyTools.createKey(FlightControllerKey.KeyLowBatteryRTHInfo)
    private val seriousLowBatteryKey = KeyTools.createKey(FlightControllerKey.KeySeriousLowBatteryWarningThreshold)
    private val lowBatteryKey = KeyTools.createKey(FlightControllerKey.KeyLowBatteryWarningThreshold)
    private val timeNeededToLandKey = KeyTools.createKey(FlightControllerKey.KeyLowBatteryRTHInfo)

    private val flightModeKey: DJIKey<FlightMode> = FlightControllerKey.KeyFlightMode.create()
    private fun getFlightMode(): FlightMode = flightModeKey.get(FlightMode.UNKNOWN)

    data class RemainingFlightTimeData(
        val remainingCharge: Int,
        val batteryNeededToLand: Int,
        val batteryNeededToGoHome: Int,
        val seriousLowBatteryThreshold: Int,
        val lowBatteryThreshold: Int,
        val flightTime: Int
    )

    private fun getRemainingFlightTimeData(): RemainingFlightTimeData {
        val goHomeInfo = goHomeAssessmentProcessor.value
        return RemainingFlightTimeData(
            chargeRemainingProcessor.value,
            goHomeInfo.batteryPercentNeededToLand,
            goHomeInfo.batteryPercentNeededToGoHome,
            seriousLowBatteryThresholdProcessor.value,
            lowBatteryThresholdProcessor.value,
            goHomeInfo.remainingFlightTime
        )
    }

    private fun setupBatteryKeyListeners() {
        KeyManager.getInstance().listen(chargeRemainingKey, this) { _, newValue ->
            chargeRemainingProcessor.onNext(newValue ?: 0)
        }

        KeyManager.getInstance().listen(goHomeAssessmentKey, this) { _, newValue ->
            goHomeAssessmentProcessor.onNext(newValue ?: LowBatteryRTHInfo())
        }

        KeyManager.getInstance().listen(seriousLowBatteryKey, this) { _, newValue ->
            seriousLowBatteryThresholdProcessor.onNext(newValue ?: 0)
        }

        KeyManager.getInstance().listen(lowBatteryKey, this) { _, newValue ->
            lowBatteryThresholdProcessor.onNext(newValue ?: 0)
        }

        KeyManager.getInstance().listen(timeNeededToLandKey, this) { _, newValue ->
            timeNeededToLandProcessor.onNext(newValue?.timeNeededToLand ?: 0)
        }
    }

    // Simple HTTP server implementation
    private inner class SimpleHttpServer(private val port: Int) {
        private var serverSocket: ServerSocket? = null
        private val executor = Executors.newFixedThreadPool(10)
        @Volatile
        private var isRunning = false

        fun start() {
            if (isRunning) return

            thread {
                try {
                    serverSocket = ServerSocket(port)
                    isRunning = true
                    Log.i("SimpleHttpServer", "Server started on port $port")

                    while (isRunning && !serverSocket!!.isClosed) {
                        try {
                            val clientSocket = serverSocket!!.accept()
                            executor.submit { handleRequest(clientSocket) }
                        } catch (e: Exception) {
                            if (isRunning) {
                                Log.e("SimpleHttpServer", "Error accepting connection: ${e.message}")
                            }
                        }
                    }
                } catch (e: Exception) {
                    Log.e("SimpleHttpServer", "Server error: ${e.message}")
                }
            }
        }

        fun stop() {
            isRunning = false
            try {
                serverSocket?.close()
                executor.shutdown()
            } catch (e: Exception) {
                Log.e("SimpleHttpServer", "Error stopping server: ${e.message}")
            }
        }

        private fun handleRequest(clientSocket: Socket) {
            try {
                val reader = BufferedReader(InputStreamReader(clientSocket.getInputStream()))
                val writer = PrintWriter(OutputStreamWriter(clientSocket.getOutputStream()), true)

                // Read the request line
                val requestLine = reader.readLine() ?: return
                val parts = requestLine.split(" ")
                if (parts.size < 3) return

                val method = parts[0]
                val uri = parts[1]

                // Read headers to get content length for POST requests
                var contentLength = 0
                var line: String?
                while (reader.readLine().also { line = it } != null && line!!.isNotEmpty()) {
                    if (line!!.startsWith("Content-Length:", ignoreCase = true)) {
                        contentLength = line!!.substring(15).trim().toIntOrNull() ?: 0
                    }
                }

                // Read POST body if present
                var postData = ""
                if (method == "POST" && contentLength > 0) {
                    val buffer = CharArray(contentLength)
                    reader.read(buffer, 0, contentLength)
                    postData = String(buffer)
                }

                // Handle the request and generate response
                val response = handleHttpRequest(method, uri, postData)

                // Send HTTP response
                writer.println("HTTP/1.1 200 OK")
                writer.println("Content-Type: text/plain")
                writer.println("Content-Length: ${response.length}")
                writer.println("Access-Control-Allow-Origin: *")
                writer.println("Access-Control-Allow-Methods: GET, POST, OPTIONS")
                writer.println("Access-Control-Allow-Headers: Content-Type")
                writer.println()
                writer.print(response)
                writer.flush()

                clientSocket.close()
            } catch (e: Exception) {
                Log.e("SimpleHttpServer", "Error handling request: ${e.message}")
                try {
                    clientSocket.close()
                } catch (ex: Exception) {
                    // Ignore
                }
            }
        }

        private fun handleHttpRequest(method: String, uri: String, postData: String): String {
            return when (method) {
                "POST" -> handlePostRequest(uri, postData)
                "GET" -> "GET requests are no longer supported. Use TCP socket on port 8081 for telemetry."
                else -> "Method Not Allowed"
            }
        }

        private fun handlePostRequest(uri: String, postData: String): String {
            return try {
                // Show toast notification on main thread to indicate request received
                mainHandler.post {
                    ToastUtils.showToast("POST request received: $uri")
                }

                // Log the request for debugging
                Log.i("DroneServer", "Handling POST request: $uri with data: $postData")

                when (uri) {
                    "/send/takeoff" -> {
                        DroneController.startTakeOff()
                        mainHandler.post {
                            ToastUtils.showToast("Takeoff command executed!")
                        }
                        "Takeoff command sent."
                    }
                    "/send/land" -> {
                        DroneController.startLanding()
                        mainHandler.post {
                            ToastUtils.showToast("Landing command executed!")
                        }
                        "Landing command sent."
                    }
                    "/send/RTH" -> {
                        DroneController.startReturnToHome()
                        mainHandler.post {
                            ToastUtils.showToast("Return to home command executed!")
                        }
                        "Return to home command sent."
                    }
                    "/send/stick" -> {
                        val cmd = postData.split(",")
                        val lx = cmd[0].toFloat()
                        val ly = cmd[1].toFloat()
                        val rx = cmd[2].toFloat()
                        val ry = cmd[3].toFloat()
                        DroneController.setStick(lx, ly, rx, ry)
                        mainHandler.post {
                            ToastUtils.showToast("Stick command: L($lx,$ly) R($rx,$ry)")
                        }
                        "Received: leftX: $lx, leftY: $ly, rightX: $rx, rightY: $ry"
                    }
                    "/send/gimbal/pitch" -> {
                        val cmd = postData.split(",")
                        val roll = cmd[0].toDouble()
                        val pitch = cmd[1].toDouble()
                        val yaw = cmd[2].toDouble()
                        val rot = GimbalAngleRotation(
                            GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                            pitch, roll, yaw, false, true, true, 0.1, false, 0
                        )
                        gimbalKey.action(rot)
                        mainHandler.post {
                            ToastUtils.showToast("Gimbal pitch: R:$roll P:$pitch Y:$yaw")
                        }
                        "Received: roll: $roll, pitch: $pitch, yaw: $yaw"
                    }
                    "/send/gimbal/yaw" -> {
                        val cmd = postData.split(",")
                        val roll = cmd[0].toDouble()
                        val pitch = cmd[1].toDouble()
                        val yaw = cmd[2].toDouble()
                        val rot = GimbalAngleRotation(
                            GimbalAngleRotationMode.ABSOLUTE_ANGLE,
                            pitch, roll, yaw, true, true, false, 0.1, false, 0
                        )
                        gimbalKey.action(rot)
                        mainHandler.post {
                            ToastUtils.showToast("Gimbal yaw: R:$roll P:$pitch Y:$yaw")
                        }
                        "Received: roll: $roll, pitch: $pitch, yaw: $yaw"
                    }
                    "/send/gotoYaw" -> {
                        val yaw = postData.split(",")[0].toDouble()
                        DroneController.gotoYaw(yaw)
                        mainHandler.post {
                            ToastUtils.showToast("Goto yaw: $yaw degrees")
                        }
                        "Received: yaw: $yaw"
                    }
                    "/send/gotoAltitude" -> {
                        val targetAltitude = postData.split(",")[0].toDouble()
                        DroneController.gotoAltitude(targetAltitude)
                        mainHandler.post {
                            ToastUtils.showToast("Goto altitude: $targetAltitude m")
                        }
                        "Received: Altitude: $targetAltitude"
                    }
                    "/send/camera/zoom" -> {
                        val targetZoom = postData.toDouble()
                        zoomKey.set(targetZoom)
                        mainHandler.post {
                            ToastUtils.showToast("Camera zoom: ${targetZoom}x")
                        }
                        "Received: zoom: $targetZoom"
                    }
                    "/send/abortMission" -> {
                        DroneController.setStick(0.0f, 0.0f, 0.0f, 0.0f)
                        DroneController.disableVirtualStick()
                        mainHandler.post {
                            ToastUtils.showToast("Mission aborted!")
                        }
                        "Received: abortMission"
                    }
                    "/send/abortAll" -> {
                        DroneController.abortAllMissions()
                        mainHandler.post {
                            ToastUtils.showToast("All missions aborted!")
                        }
                        "Received: abortAll"
                    }
                    "/send/enableVirtualStick" -> {
                        DroneController.enableVirtualStick()
                        mainHandler.post {
                            ToastUtils.showToast("Virtual stick enabled!")
                        }
                        "Received: enableVirtualStick"
                    }
                    "/send/camera/startRecording" -> {
                        startRecording.action()
                        mainHandler.post {
                            ToastUtils.showToast("Camera recording started!")
                        }
                        "Received: camera start recording"
                    }
                    "/send/camera/stopRecording" -> {
                        stopRecording.action()
                        mainHandler.post {
                            ToastUtils.showToast("Camera recording stopped!")
                        }
                        "Received: camera stop recording"
                    }
                    "/send/gotoWP" -> {
                        val cmd = postData.split(",")
                        if (cmd.size < 3) {
                            return "Invalid input. Expected format: lat,lon,alt"
                        }
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()
                        DroneController.gotoWP(latitude, longitude, altitude)
                        mainHandler.post {
                            ToastUtils.showToast("Goto waypoint: $latitude,$longitude,$altitude")
                        }
                        "Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude"
                    }
                    "/send/gotoWPwithPID" -> {
                        val cmd = postData.split(",")
                        if (cmd.size < 5) {
                            return "Invalid input. Expected format: lat,lon,alt,yaw,maxSpeed"
                        }
                        val latitude = cmd[0].toDouble()
                        val longitude = cmd[1].toDouble()
                        val altitude = cmd[2].toDouble()
                        val yaw = cmd[3].toDouble()
                        val maxSpeed = cmd[4].toDouble()
                        DroneController.navigateToWaypointWithPID(latitude, longitude, altitude, yaw, maxSpeed)
                        "Waypoint command received: Latitude=$latitude, Longitude=$longitude, Altitude=$altitude, Yaw=$yaw, MaxSpeed=$maxSpeed"
                    }
                    "/send/navigateTrajectory" -> {
                        Log.d("DroneServer", "Received trajectory data: $postData")
                        val segments = postData.split(";").map { it.trim() }.filter { it.isNotEmpty() }
                        if (segments.isEmpty()) {
                            return "Invalid input. Expected at least one waypoint and a yaw."
                        }
                        val lastSegment = segments.last().split(",").map { it.trim() }
                        if (lastSegment.size < 4) {
                            return "Invalid input. The last segment must have lat,lon,alt,yaw."
                        }
                        val finalLatitude = lastSegment[0].toDouble()
                        val finalLongitude = lastSegment[1].toDouble()
                        val finalAltitude = lastSegment[2].toDouble()
                        val finalYaw = lastSegment[3].toDouble()
                        val waypoints = mutableListOf<Triple<Double, Double, Double>>()
                        for (i in 0 until segments.size - 1) {
                            val parts = segments[i].split(",").map { it.trim() }
                            if (parts.size < 3) {
                                return "Invalid input at segment $i: expected lat,lon,alt"
                            }
                            val lat = parts[0].toDouble()
                            val lon = parts[1].toDouble()
                            val alt = parts[2].toDouble()
                            waypoints.add(Triple(lat, lon, alt))
                        }
                        waypoints.add(Triple(finalLatitude, finalLongitude, finalAltitude))
                        Log.d("DroneServer", "Navigating trajectory with ${waypoints.size} waypoints, finalYaw: $finalYaw")
                        DroneController.navigateTrajectory(waypoints, finalYaw)
                        "Trajectory command received. Waypoints=${waypoints.size}, FinalYaw=$finalYaw"
                    }
                    // --- New endpoints ---
                    "/send/navigateTrajectoryDJINative" -> {
                        // Expect: "speed;lat,lon,alt;lat,lon,alt;..."
                        val segments = postData.split(";").map { it.trim() }.filter { it.isNotEmpty() }
                        if (segments.size < 3) return "Invalid input. Need speed and at least 2 waypoints: speed;lat,lon,alt;..."

                        val trajectorySpeed = segments[0].toDoubleOrNull()
                            ?: return "Invalid input. Speed must be a number."

                        val waypoints = mutableListOf<Triple<Double, Double, Double>>()
                        for (i in 1 until segments.size) {
                            val s = segments[i]
                            val parts = s.split(",").map { it.trim() }
                            if (parts.size < 3) return "Invalid input at segment ${i - 1}: expected lat,lon,alt"
                            val lat = parts[0].toDouble()
                            val lon = parts[1].toDouble()
                            val alt = parts[2].toDouble()
                            waypoints.add(Triple(lat, lon, alt))
                        }

                        if (waypoints.size < 2) {
                            return "Invalid input. Need at least 2 waypoints."
                        }

                        DroneController.navigateTrajectoryNative(waypoints, trajectorySpeed)
                        mainHandler.post { ToastUtils.showToast("DJI native mission started (${waypoints.size} wps at ${trajectorySpeed}m/s)") }
                        "DJI native mission requested with ${waypoints.size} waypoints at ${trajectorySpeed}m/s"
                    }
                    "/send/abort/DJIMission" -> {
                        DroneController.endMission()
                        mainHandler.post { ToastUtils.showToast("End mission requested") }
                        "Mission stop requested"
                    }
                    "/send/setRTHAltitude" -> {
                        val altitude = postData.toIntOrNull()
                        if (altitude != null) {
                            DroneController.setRTHAltitude(altitude)
                            mainHandler.post {
                                ToastUtils.showToast("Setting RTH altitude to $altitude m")
                            }
                            "RTH altitude set to $altitude m"
                        } else {
                            "Invalid altitude value"
                        }
                    }
                    else -> "Not Found"
                }
            } catch (e: Exception) {
                Log.e("DroneServer", "Error processing POST request: ${e.message}", e)
                "Error processing request: ${e.message}"
            }
        }
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        binding = FragVirtualStickPageBinding.inflate(inflater, container, false)
        return binding?.root
    }

    @SuppressLint("SetTextI18n")
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        // Initialize DroneController with required ViewModels
        DroneController.init(basicAircraftControlVM, virtualStickVM)

        binding?.widgetHorizontalSituationIndicator?.setSimpleModeEnable(false)

        // Display device IP address
        displayDeviceIpAddress()

        // Add battery level TextView
        addBatteryLevelDisplay()

        // Set up key listeners
        setupBatteryKeyListeners()

        // Add low battery RTH info TextViews
        addLowBatteryRTHInfoDisplay()

        // Update distance to home display
        updateDistanceToHomeDisplay()

        // Set up a periodic update for distance to home
        val distanceUpdateRunnable = object : Runnable {
            override fun run() {
                updateDistanceToHomeDisplay()
                mainHandler.postDelayed(this, 1000) // Update every second
            }
        }
        mainHandler.post(distanceUpdateRunnable)

        initBtnClickListener()
        
        // Start WebRTC stream by default (instead of RTSP)
        setupAndStartWebRTCStream()
        
        virtualStickVM.listenRCStick()
        virtualStickVM.currentSpeedLevel.observe(viewLifecycleOwner) {
            updateVirtualStickInfo()
        }
        virtualStickVM.useRcStick.observe(viewLifecycleOwner) {
            updateVirtualStickInfo()
        }
        virtualStickVM.currentVirtualStickStateInfo.observe(viewLifecycleOwner) {
            updateVirtualStickInfo()
        }
        virtualStickVM.stickValue.observe(viewLifecycleOwner) {
            updateVirtualStickInfo()
        }
        virtualStickVM.virtualStickAdvancedParam.observe(viewLifecycleOwner) {
            updateVirtualStickInfo()
        }
        simulatorVM.simulatorStateSb.observe(viewLifecycleOwner) {
            binding?.simulatorStateInfoTv?.text = it
        }
        liveStreamVM.streamQuality.observe(viewLifecycleOwner) { it ->
            // Only update if in RTSP mode
            if (!isWebRTCMode) {
                "RTSP: $it".also { binding?.streamQualityInfoTv?.text = it }
            }
        }

        startServers()

        // Initialize camera stream
        initCameraStream()
        
        // Display available zoom ratios
        displayCameraZoomRatios()
        
        // Listen for zoom ratios changes
        KeyManager.getInstance().listen(zoomRatiosRangeKey, this) { _, _ ->
            displayCameraZoomRatios()
        }
    }

    private fun initBtnClickListener() {
        binding?.btnEnableVirtualStick?.setOnClickListener {
            virtualStickVM.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("enableVirtualStick success.")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("enableVirtualStick error,$error")
                }
            })
        }
        binding?.btnDisableVirtualStick?.setOnClickListener {
            virtualStickVM.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("disableVirtualStick success.")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("disableVirtualStick error,${error})")
                }
            })
        }
        binding?.btnSetVirtualStickSpeedLevel?.setOnClickListener {
            val speedLevels = doubleArrayOf(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
            initPopupNumberPicker(Helper.makeList(speedLevels)) {
                virtualStickVM.setSpeedLevel(speedLevels[indexChosen[0]])
                resetIndex()
            }
        }
        binding?.btnTakeOff?.setOnClickListener {
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
        binding?.btnLanding?.setOnClickListener {
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
        binding?.btnUseRcStick?.setOnClickListener {
            virtualStickVM.useRcStick.value = virtualStickVM.useRcStick.value != true
            if (virtualStickVM.useRcStick.value == true) {
                ToastUtils.showToast(
                    "After it is turned on," +
                            "the joystick value of the RC will be used as the left/ right stick value"
                )
            }
        }
        binding?.btnSetVirtualStickAdvancedParam?.setOnClickListener {
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
        binding?.btnSendVirtualStickAdvancedParam?.setOnClickListener {
            virtualStickVM.virtualStickAdvancedParam.value?.let {
                virtualStickVM.sendVirtualStickAdvancedParam(it)
            }
        }
        binding?.btnEnableVirtualStickAdvancedMode?.setOnClickListener {
            virtualStickVM.enableVirtualStickAdvancedMode()
        }
        binding?.btnDisableVirtualStickAdvancedMode?.setOnClickListener {
            virtualStickVM.disableVirtualStickAdvancedMode()
        }
        
        // Toggle between WebRTC and RTSP streaming
        binding?.btnToggleStreamMode?.setOnClickListener {
            toggleStreamingMode()
        }
    }

    private fun updateVirtualStickInfo() {
        val builder = StringBuilder()
        builder.append("Speed level:").append(virtualStickVM.currentSpeedLevel.value)
        builder.append("\n")
        builder.append("Use rc stick as virtual stick:").append(virtualStickVM.useRcStick.value)
        builder.append("\n")
        builder.append("Is virtual stick enable:").append(virtualStickVM.currentVirtualStickStateInfo.value?.state?.isVirtualStickEnable)
        builder.append("\n")
        builder.append("Current control permission owner:").append(virtualStickVM.currentVirtualStickStateInfo.value?.state?.currentFlightControlAuthorityOwner)
        builder.append("\n")
        builder.append("Change reason:").append(virtualStickVM.currentVirtualStickStateInfo.value?.reason)
        builder.append("\n")
        builder.append("Rc stick value:").append(virtualStickVM.stickValue.value?.toString())
        builder.append("\n")
        builder.append("Is virtual stick advanced mode enable:").append(virtualStickVM.currentVirtualStickStateInfo.value?.state?.isVirtualStickAdvancedModeEnabled)
        builder.append("\n")
        builder.append("Virtual stick advanced mode param:").append(virtualStickVM.virtualStickAdvancedParam.value?.toJson())
        builder.append("\n")
        mainHandler.post {
            binding?.virtualStickInfoTv?.text = builder.toString()
        }
    }

    // ==================== WebRTC Streaming ====================
    
    private fun setupAndStartWebRTCStream() {
        isWebRTCMode = true
        updateStreamingModeUI()
        
        webRTCStreamer = WebRTCStreamer(
            context = requireContext(),
            cameraIndex = cameraIndex,
            signalingPort = wEBRTCPORT,
            options = WebRTCMediaOptions()
        )
        
        webRTCStreamer?.listener = object : WebRTCStreamer.WebRTCStreamerListener {
            @SuppressLint("SetTextI18n")
            override fun onServerStarted(ip: String, port: Int) {
                mainHandler.post {
                    ToastUtils.showToast("WebRTC server started at ws://$ip:$port")
                    binding?.streamQualityInfoTv?.text = "WebRTC: ws://$ip:$port"
                }
            }

            @SuppressLint("SetTextI18n")
            override fun onServerStopped() {
                mainHandler.post {
                    binding?.streamQualityInfoTv?.text = "WebRTC: Stopped"
                }
            }

            @SuppressLint("SetTextI18n")
            override fun onServerError(error: String) {
                mainHandler.post {
                    ToastUtils.showToast("WebRTC error: $error")
                    binding?.streamQualityInfoTv?.text = "WebRTC: Error"
                }
            }

            @SuppressLint("SetTextI18n")
            override fun onClientConnected(clientId: String, totalClients: Int) {
                mainHandler.post {
                    binding?.streamQualityInfoTv?.text = "WebRTC: $totalClients client(s)"
                }
            }

            @SuppressLint("SetTextI18n")
            override fun onClientDisconnected(clientId: String, totalClients: Int) {
                mainHandler.post {
                    binding?.streamQualityInfoTv?.text = "WebRTC: $totalClients client(s)"
                }
            }
        }
        
        webRTCStreamer?.start()
        Log.i("VirtualStickFragment", "WebRTC streamer started on port $wEBRTCPORT")
    }
    
    private fun stopWebRTCStream() {
        webRTCStreamer?.stop()
        webRTCStreamer = null
        Log.i("VirtualStickFragment", "WebRTC streamer stopped")
    }
    
    // ==================== RTSP Streaming ====================

    private fun setupAndStartRtspStream() {
        isWebRTCMode = false
        updateStreamingModeUI()
        
        // Set RTSP configuration with the specified parameters
        liveStreamVM.setRTSPConfig(
            "aaa", // username
            "aaa", // password
            8554   // port
        )

        // Set stream quality to FULL_HD
        liveStreamVM.setLiveStreamQuality(StreamQuality.ORIGINAL)

        // Start the stream
        liveStreamVM.startStream(object : CommonCallbacks.CompletionCallback {
            @SuppressLint("SetTextI18n")
            override fun onSuccess() {
                mainHandler.post {
                    ToastUtils.showToast("RTSP stream started successfully")
                    binding?.streamQualityInfoTv?.text = "RTSP: rtsp://$deviceIp:8554"
                }
            }

            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("Failed to start RTSP stream: ${error.description()}")
            }
        })
    }

    private fun stopRtspStream() {
        if (liveStreamVM.isStreaming()) {
            liveStreamVM.stopStream(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("RTSP stream stopped successfully")
                }

                override fun onFailure(error: IDJIError) {
                    ToastUtils.showToast("Failed to stop RTSP stream: ${error.description()}")
                }
            })
        }
    }
    
    // ==================== Stream Toggle ====================
    
    private fun toggleStreamingMode() {
        if (isWebRTCMode) {
            // Switch to RTSP
            stopWebRTCStream()
            setupAndStartRtspStream()
        } else {
            // Switch to WebRTC
            stopRtspStream()
            setupAndStartWebRTCStream()
        }
    }
    
    private fun updateStreamingModeUI() {
        mainHandler.post {
            binding?.btnToggleStreamMode?.text = if (isWebRTCMode) {
                "Switch to RTSP"
            } else {
                "Switch to WebRTC"
            }
        }
    }
    
    @SuppressLint("SetTextI18n")
    private fun displayCameraZoomRatios() {
        try {
            val zoomRatiosRange = zoomRatiosRangeKey.get()
            
            if (zoomRatiosRange != null) {
                // Build a display string of available zoom ratios
                val zoomRatiosText = StringBuilder("Camera Zoom Ratios:\n")
                
                // ZoomRatiosRange contains the zoom range information
                val itemStr = zoomRatiosRange.toString()
                zoomRatiosText.append(itemStr)
                
                Log.d(TAG, "Zoom ratio range: $itemStr")
                
                mainHandler.post {
                    binding?.cameraZoomRatiosTv?.text = zoomRatiosText.toString()
                }
            } else {
                mainHandler.post {
                    binding?.cameraZoomRatiosTv?.text = "Camera Zoom Ratios: Not available"
                }
                Log.w(TAG, "Zoom ratios range is null")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error getting zoom ratios: ${e.message}", e)
            mainHandler.post {
                binding?.cameraZoomRatiosTv?.text = "Camera Zoom Ratios: Error - ${e.message}"
            }
        }
    }


    private val gimbalKey: DJIKey.ActionKey<GimbalAngleRotation, EmptyMsg> =
        GimbalKey.KeyRotateByAngle.create()
    private val zoomKey: DJIKey<Double> = CameraKey.KeyCameraZoomRatios.create()
    private val zoomRatiosRangeKey = CameraKey.KeyCameraZoomRatiosRange.create()
    private val startRecording: DJIKey.ActionKey<EmptyMsg, EmptyMsg> = CameraKey.KeyStartRecord.create()
    private val stopRecording: DJIKey.ActionKey<EmptyMsg, EmptyMsg> = CameraKey.KeyStopRecord.create()
    private val isRecording: DJIKey<Boolean> = CameraKey.KeyIsRecording.create()

    private val location3DKey: DJIKey<LocationCoordinate3D> =
        FlightControllerKey.KeyAircraftLocation3D.create()

    private fun getLocation3D(): LocationCoordinate3D {
        return location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))
    }

    private val satelliteCountKey: DJIKey<Int> = FlightControllerKey.KeyGPSSatelliteCount.create()
    private fun getSatelliteCount(): Int = satelliteCountKey.get(-1)

    private val gimbalJointAttitudeKey: DJIKey<Attitude> = GimbalKey.KeyGimbalJointAttitude.create()
    private fun getJointAttitude(): Attitude = gimbalJointAttitudeKey.get(Attitude(0.0, 0.0, 0.0))

    private val gimbalAttitudeKey: DJIKey<Attitude> = GimbalKey.KeyGimbalAttitude.create()
    private fun getGimbalAttitudeKey(): Attitude = gimbalAttitudeKey.get(Attitude(0.0, 0.0, 0.0))

    private val compassHeadKey: DJIKey<Double> = FlightControllerKey.KeyCompassHeading.create()
    private fun getHeading(): Double {
        return (compassHeadKey.get(0.0)).toDouble()
    }

    private val homeLocationKey: DJIKey<LocationCoordinate2D> = FlightControllerKey.KeyHomeLocation.create()
    private fun getLocationHome(): LocationCoordinate2D = homeLocationKey.get(LocationCoordinate2D())

    private val flightSpeed: DJIKey<Velocity3D> = FlightControllerKey.KeyAircraftVelocity.create()
    private fun getSpeed(): Velocity3D = flightSpeed.get(Velocity3D(0.0, 0.0, 0.0))

    private val attitudeKey: DJIKey<Attitude> = FlightControllerKey.KeyAircraftAttitude.create()
    private fun getAttitude(): Attitude = attitudeKey.get(Attitude(0.0, 0.0, 0.0))

    private val cameraZoomFocalLengthKey: DJIKey<Int> = CameraKey.KeyCameraZoomFocalLength.create()
    private fun getCameraZoomFocalLength(): Int = cameraZoomFocalLengthKey.get(-1)

    private val cameraOpticalFocalLengthKey: DJIKey<Int> =
        CameraKey.KeyCameraOpticalZoomFocalLength.create()
    private fun getCameraOpticalFocalLength(): Int = cameraOpticalFocalLengthKey.get(-1)

    private val cameraHybridFocalLengthKey: DJIKey<Int> =
        CameraKey.KeyCameraHybridZoomFocalLength.create()
    private fun getCameraHybridFocalLength(): Int = cameraHybridFocalLengthKey.get(-1)

    private val batteryKey: DJIKey<Int> = BatteryKey.KeyChargeRemainingInPercent.create()
    private fun getBatteryLevel(): Int = batteryKey.get(-1)

    private fun getTimeNeededToGoHome(): Int = goHomeAssessmentProcessor.value.timeNeededToGoHome
    private fun getTimeNeededToLand(): Int = timeNeededToLandProcessor.value

    // Get device IP address
    private val deviceIp: String? by lazy {
        getDeviceIpAddress()
    }

    // Fixed to remove unused context parameter
    private fun getDeviceIpAddress(): String? {
        try {
            // Retrieve all network interfaces
            val interfaces = NetworkInterface.getNetworkInterfaces()
            for (networkInterface in Collections.list(interfaces)) {
                // Skip inactive interfaces and loopback interfaces
                if (!networkInterface.isUp || networkInterface.isLoopback) continue

                // Iterate through all IP addresses assigned to the interface
                val addresses = networkInterface.inetAddresses
                for (address in Collections.list(addresses)) {
                    // Check if the address is IPv4 and not a loopback address
                    if (address is Inet4Address && !address.isLoopbackAddress) {
                        return address.hostAddress
                    }
                }
            }
        } catch (e: Exception) {
            Log.e("VirtualStickFragment", "Error getting IP address: ${e.message}")
        }
        return null
    }

    private fun isPortInUse(port: Int): Boolean {
        return try {
            ServerSocket(port).close()
            false
        } catch (e: IOException) {
            true
        }
    }

    private fun displayDeviceIpAddress() {
        val ipAddress = deviceIp ?: "Not Available"
        val displayText = "Device IP: $ipAddress"

        mainHandler.post {
            binding?.deviceIpTv?.text = displayText
        }

        // Also log the IP address for debugging
        Log.i("VirtualStickFragment", "Device IP Address: $ipAddress")

        // Show a toast with the IP address when the fragment loads
        ToastUtils.showToast("Device IP: $ipAddress")
    }

    private fun startServers() {
        if (!isPortInUse(8080)) {
            try {
                httpServer = SimpleHttpServer(8080)
                httpServer?.start()
                Log.i("VirtualStickFragment", "HTTP server started on $deviceIp:8080")
            } catch (e: Exception) {
                Log.e("VirtualStickFragment", "Error starting HTTP server: ${e.message}")
                ToastUtils.showToast("Failed to start HTTP server: ${e.message}")
            }
        }
        if (!isPortInUse(8081)) {
            try {
                telemetryServer = TelemetryServer(8081, ::getTelemetryJson)
                telemetryServer?.start()
                Log.i("VirtualStickFragment", "Telemetry server started on $deviceIp:8081")
            } catch (e: Exception) {
                Log.e("VirtualStickFragment", "Error starting telemetry server: ${e.message}")
                ToastUtils.showToast("Failed to start telemetry server: ${e.message}")
            }
        }
    }

    @SuppressLint("SetTextI18n")
    private fun addBatteryLevelDisplay() {
        // Set up a periodic update for battery level
        val batteryUpdateRunnable = object : Runnable {
            override fun run() {
                val currentBatteryLevel = getBatteryLevel()
                mainHandler.post {
                    "Battery Level: $currentBatteryLevel%".also { binding?.batteryLevelTv?.text = it }
                }
                mainHandler.postDelayed(this, 1000) // Update every second
            }
        }

        // Start the periodic updates
        mainHandler.post(batteryUpdateRunnable)
    }

    private fun addLowBatteryRTHInfoDisplay() {
        // Set up a periodic update for low battery RTH info
        val lowBatteryRTHInfoUpdateRunnable = object : Runnable {
            override fun run() {
                updateLowBatteryRTHInfoDisplay()
                mainHandler.postDelayed(this, 1000) // Update every second
            }
        }

        // Start the periodic updates
        mainHandler.post(lowBatteryRTHInfoUpdateRunnable)
    }

    @SuppressLint("SetTextI18n")
    private fun updateLowBatteryRTHInfoDisplay() {
        val rftData = getRemainingFlightTimeData()
        mainHandler.post {
            binding?.remainingFlightTimeTv?.text =
                "Remaining Flight Time: ${rftData.flightTime} sec"
            binding?.timeNeededToGoHomeTv?.text =
                "Time Needed to Land: ${getTimeNeededToGoHome() + getTimeNeededToLand()} sec"
        }
    }

    @SuppressLint("SetTextI18n", "DefaultLocale")
    private fun updateDistanceToHomeDisplay() {
        val current = getLocation3D()
        val home = getLocationHome()
        val distance = DroneController.calculateDistance(current.latitude, current.longitude, home.latitude, home.longitude)

        mainHandler.post {
            binding?.distanceToHomeTv?.text = "Distance to Home: ${String.format("%.2f", distance)} m"
        }
    }

    private fun initCameraStream() {
        // Initialize camera stream variables
        cameraIndex = ComponentIndexType.LEFT_OR_MAIN
        cameraStreamWidth = 640
        cameraStreamHeight = 480
        cameraStreamScaleType = ICameraStreamManager.ScaleType.CENTER_INSIDE
        isVideoFeedEnabled = true

        // Set up the camera stream surface
        binding?.cameraStreamSurfaceView?.holder?.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                cameraStreamSurface = holder.surface
                startCameraStream()
            }

            override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
                cameraStreamWidth = width
                cameraStreamHeight = height
                startCameraStream()
            }

            override fun surfaceDestroyed(holder: SurfaceHolder) {
                stopCameraStream()
                cameraStreamSurface = null
            }
        })
    }

    private fun startCameraStream() {
        if (cameraStreamSurface != null && isVideoFeedEnabled) {
            cameraStreamManager.putCameraStreamSurface(
                cameraIndex,
                cameraStreamSurface!!,
                cameraStreamWidth,
                cameraStreamHeight,
                cameraStreamScaleType
            )
            Log.i("CameraStream", "Camera stream started successfully")
        }
    }

    private fun stopCameraStream() {
        if (cameraStreamSurface != null) {
            cameraStreamManager.removeCameraStreamSurface(cameraStreamSurface!!)
            Log.i("CameraStream", "Camera stream stopped successfully")
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        stopWebRTCStream()
        stopRtspStream()
        httpServer?.stop()
        telemetryServer?.stop()
        stopCameraStream()
        KeyManager.getInstance().cancelListen(this)
    }

    private fun isHomeSet(): Boolean {
        if (isHomePointSetLatch) {
            return true
        }

        val isFlyingKey: DJIKey<Boolean> = FlightControllerKey.KeyIsFlying.create()
        val isFlying = isFlyingKey.get(false)

        if (!isFlying) {
            val home = getLocationHome()
            if (home.latitude != 0.0 && home.longitude != 0.0) {
                val current = getLocation3D()
                val distance = DroneController.calculateDistance(current.latitude, current.longitude, home.latitude, home.longitude)
                if (distance < 0.5) {
                    isHomePointSetLatch = true
                    return true
                }
            }
        }
        return isHomePointSetLatch
    }

    //region --- Telemetry JSON ---
    private fun getTelemetryJson(): String {
        val rftData = getRemainingFlightTimeData()
        val timeNeededToGoHome = getTimeNeededToGoHome().toString()
        val timeNeededToLand = getTimeNeededToLand().toString()
        val totalTime = (getTimeNeededToGoHome() + getTimeNeededToLand()).toString()
        val maxRadiusCanFlyAndGoHome = goHomeAssessmentProcessor.value.maxRadiusCanFlyAndGoHome.toString()
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
        val homeLocation = getLocationHome().toString()
        val distanceToHome = DroneController.calculateDistance(getLocation3D().latitude, getLocation3D().longitude, getLocationHome().latitude, getLocationHome().longitude).toString()
        val waypointReached = DroneController.isWaypointReached()
        val intermediaryWaypointReached = DroneController.isIntermediaryWaypointReached()
        val yawReached = DroneController.isYawReached()
        val altitudeReached = DroneController.isAltitudeReached()
        val isRecording = isRecording.get().toString()
        val homeSet = isHomeSet().toString()
        val flightMode = "\"${getFlightMode().name}\""

        // Extract values from rftData
        val remainingCharge = rftData.remainingCharge.toString()
        val batteryNeededToLand = rftData.batteryNeededToLand.toString()
        val batteryNeededToGoHome = rftData.batteryNeededToGoHome.toString()
        val seriousLowBatteryThreshold = rftData.seriousLowBatteryThreshold.toString()
        val lowBatteryThreshold = rftData.lowBatteryThreshold.toString()
        val remainingFlightTime = rftData.flightTime.toString()

        return "{\"speed\":$speed,\"heading\":$heading,\"attitude\":$attitude,\"location\":$location," +
                "\"gimbalAttitude\":$gimbalAttitude,\"gimbalJointAttitude\":$gimbalJointAttitude," +
                "\"zoomFl\":$zoomFl,\"hybridFl\":$hybridFl,\"opticalFl\":$opticalFl," +
                "\"zoomRatio\":$zoomRatio,\"batteryLevel\":$batteryLevel,\"satelliteCount\":$satelliteCount," +
                "\"homeLocation\":$homeLocation,\"distanceToHome\":$distanceToHome," +
                "\"waypointReached\":$waypointReached,\"intermediaryWaypointReached\":$intermediaryWaypointReached," +
                "\"yawReached\":$yawReached,\"altitudeReached\":$altitudeReached,\"isRecording\":$isRecording," +
                "\"homeSet\":$homeSet,\"remainingFlightTime\":$remainingFlightTime," +
                "\"timeNeededToGoHome\":$timeNeededToGoHome,\"timeNeededToLand\":$timeNeededToLand," +
                "\"totalTime\":$totalTime,\"maxRadiusCanFlyAndGoHome\":$maxRadiusCanFlyAndGoHome," +
                "\"remainingCharge\":$remainingCharge,\"batteryNeededToLand\":$batteryNeededToLand," +
                "\"batteryNeededToGoHome\":$batteryNeededToGoHome,\"seriousLowBatteryThreshold\":$seriousLowBatteryThreshold," +
                "\"lowBatteryThreshold\":$lowBatteryThreshold,\"flightMode\":$flightMode}"
    }
    //endregion
}
