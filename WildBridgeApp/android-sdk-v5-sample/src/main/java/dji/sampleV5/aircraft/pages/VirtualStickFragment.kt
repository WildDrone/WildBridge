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
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.Collections
import dji.v5.ux.core.util.DataProcessor
import dji.sdk.keyvalue.key.KeyTools
import dji.sampleV5.aircraft.controller.DroneController
import dji.sampleV5.aircraft.logger.WildBridgeFlightLogger

// Import for custom HTTP server implementation
import dji.v5.manager.KeyManager

import dji.sdk.keyvalue.value.flightcontroller.FlightMode

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
    private var isHomePointSetLatch = false

    // Periodic flight-log telemetry snapshot (every 5 s, only while a session is active)
    private var telemetryLogRunnable: Runnable? = null
    private var distanceUpdateRunnable: Runnable? = null
    private var batteryUpdateRunnable: Runnable? = null
    private var lowBatteryRTHInfoUpdateRunnable: Runnable? = null

    private var isRtspStreaming = false

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

        // ---- Manual Override checkbox setup ----
        setupManualOverrideCheckbox()

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
        distanceUpdateRunnable = object : Runnable {
            override fun run() {
                updateDistanceToHomeDisplay()
                mainHandler.postDelayed(this, 1000) // Update every second
            }
        }
        mainHandler.post(distanceUpdateRunnable!!)

        // Snapshot telemetry to the flight log every 5 seconds while a session is open.
        telemetryLogRunnable = object : Runnable {
            override fun run() {
                if (WildBridgeFlightLogger.isSessionActive) {
                    WildBridgeFlightLogger.logTelemetry(getTelemetryJson())
                }
                mainHandler.postDelayed(this, 5_000)
            }
        }
        mainHandler.postDelayed(telemetryLogRunnable!!, 5_000)

        initBtnClickListener()
        
        updateStreamingModeUI()
        
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
            if (isRtspStreaming) {
                "RTSP: $it".also { binding?.streamQualityInfoTv?.text = it }
            }
        }

        // Initialize camera stream
        initCameraStream()
        
        // Display available zoom ratios
        displayCameraZoomRatios()
        
        // Listen for zoom ratios changes
        KeyManager.getInstance().listen(zoomRatiosRangeKey, this) { _, _ ->
            displayCameraZoomRatios()
        }
    }

    // ==================== Manual Override Checkbox ====================

    private fun setupManualOverrideCheckbox() {
        // Sync checkbox with current state (e.g. after rotation)
        updateManualOverrideUI()

        // When the checkbox is toggled by the user:
        // - Checking it does nothing extra (it's auto-checked on activation)
        // - UN-checking it clears the manual override latch
        binding?.cbManualOverride?.setOnCheckedChangeListener { _, isChecked ->
            if (!isChecked) {
                // User explicitly deactivated manual override
                DroneController.deactivateManualOverride()
                updateManualOverrideUI()
            }
            // Note: checking it manually is a no-op — it activates automatically via RC sticks
        }

        // Register listener so DroneController can notify us when override triggers automatically
        DroneController.manualOverrideListener = object : DroneController.ManualOverrideListener {
            override fun onManualOverrideActivated() {
                mainHandler.post { updateManualOverrideUI() }
            }
        }
    }

    @SuppressLint("SetTextI18n")
    private fun updateManualOverrideUI() {
        val isActive = DroneController.isManualOverrideActive
        binding?.cbManualOverride?.let { cb ->
            // Set checked state without triggering the listener
            cb.setOnCheckedChangeListener(null)
            cb.isChecked = isActive
            cb.text = if (isActive) "\u26a0 Manual" else "Manual"
            cb.setTextColor(if (isActive) 0xFFFF0000.toInt() else 0xFFFFFFFF.toInt())
            cb.setBackgroundColor(if (isActive) 0x33FF0000 else 0x00000000)
            // Re-attach listener
            cb.setOnCheckedChangeListener { _, isChecked ->
                if (!isChecked) {
                    DroneController.deactivateManualOverride()
                    updateManualOverrideUI()
                }
            }
        }
    }

    // ==================== End Manual Override Checkbox ====================

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
        
        // Toggle RTSP streaming for the legacy virtual-stick sample page.
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

    // ==================== RTSP Streaming ====================

    private fun setupAndStartRtspStream() {
        isRtspStreaming = true
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
        isRtspStreaming = false
        updateStreamingModeUI()
    }
    
    // ==================== Stream Toggle ====================
    
    private fun toggleStreamingMode() {
        if (isRtspStreaming) {
            stopRtspStream()
        } else {
            setupAndStartRtspStream()
        }
    }
    
    private fun updateStreamingModeUI() {
        mainHandler.post {
            binding?.btnToggleStreamMode?.text = if (isRtspStreaming) {
                "Stop RTSP"
            } else {
                "Start RTSP"
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

    @SuppressLint("SetTextI18n")
    private fun addBatteryLevelDisplay() {
        // Set up a periodic update for battery level
        batteryUpdateRunnable = object : Runnable {
            override fun run() {
                val currentBatteryLevel = getBatteryLevel()
                mainHandler.post {
                    "Battery Level: $currentBatteryLevel%".also { binding?.batteryLevelTv?.text = it }
                }
                mainHandler.postDelayed(this, 1000) // Update every second
            }
        }

        // Start the periodic updates
        mainHandler.post(batteryUpdateRunnable!!)
    }

    private fun addLowBatteryRTHInfoDisplay() {
        // Set up a periodic update for low battery RTH info
        lowBatteryRTHInfoUpdateRunnable = object : Runnable {
            override fun run() {
                updateLowBatteryRTHInfoDisplay()
                mainHandler.postDelayed(this, 1000) // Update every second
            }
        }

        // Start the periodic updates
        mainHandler.post(lowBatteryRTHInfoUpdateRunnable!!)
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
        stopRtspStream()
        stopCameraStream()
        distanceUpdateRunnable?.let { mainHandler.removeCallbacks(it) }
        distanceUpdateRunnable = null
        telemetryLogRunnable?.let { mainHandler.removeCallbacks(it) }
        telemetryLogRunnable = null
        batteryUpdateRunnable?.let { mainHandler.removeCallbacks(it) }
        batteryUpdateRunnable = null
        lowBatteryRTHInfoUpdateRunnable?.let { mainHandler.removeCallbacks(it) }
        lowBatteryRTHInfoUpdateRunnable = null
        DroneController.manualOverrideListener = null
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
        val waypointSeq = DroneController.getWaypointSeq()
        val intermediaryWaypointReached = DroneController.isIntermediaryWaypointReached()
        val yawReached = DroneController.isYawReached()
        val yawSeq = DroneController.getYawSeq()
        val altitudeReached = DroneController.isAltitudeReached()
        val altitudeSeq = DroneController.getAltitudeSeq()
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
                "\"waypointReached\":$waypointReached,\"waypointSeq\":$waypointSeq,\"intermediaryWaypointReached\":$intermediaryWaypointReached," +
                "\"yawReached\":$yawReached,\"yawSeq\":$yawSeq,\"altitudeReached\":$altitudeReached,\"altitudeSeq\":$altitudeSeq,\"isRecording\":$isRecording," +
                "\"homeSet\":$homeSet,\"remainingFlightTime\":$remainingFlightTime," +
                "\"timeNeededToGoHome\":$timeNeededToGoHome,\"timeNeededToLand\":$timeNeededToLand," +
                "\"totalTime\":$totalTime,\"maxRadiusCanFlyAndGoHome\":$maxRadiusCanFlyAndGoHome," +
                "\"remainingCharge\":$remainingCharge,\"batteryNeededToLand\":$batteryNeededToLand," +
                "\"batteryNeededToGoHome\":$batteryNeededToGoHome,\"seriousLowBatteryThreshold\":$seriousLowBatteryThreshold," +
                "\"lowBatteryThreshold\":$lowBatteryThreshold,\"flightMode\":$flightMode," +
                "\"isManualOverrideActive\":${DroneController.isManualOverrideActive}}"
    }
    //endregion
}
