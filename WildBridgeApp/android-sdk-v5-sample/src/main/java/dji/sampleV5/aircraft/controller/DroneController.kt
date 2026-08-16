package dji.sampleV5.aircraft.controller

import android.os.Handler
import android.os.Looper
import dji.sampleV5.aircraft.DroneControlProfiles
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

    // ==================== Manual Override System ====================
    // When true, the pilot has taken manual control via RC sticks.
    // This flag latches ON automatically when RC stick input exceeds the deadzone,
    // and only clears when the user explicitly deactivates it (via checkbox or HTTP).
    // While active, all autonomous HTTP commands (waypoints, trajectories, etc.) are rejected.
    @Volatile
    var isManualOverrideActive = false
        private set

    // RC stick deadzone threshold [0..660]. DJI RC sticks report ±660.
    // 200 ≈ 30 % deflection — requires a clear, deliberate push to activate
    // override, making accidental triggering from calibration drift or small
    // incidental touches virtually impossible.
    const val RC_STICK_DEADZONE = 200

    // Waypoint acceptance thresholds — shared across all control loops
    const val WP_ACCEPT_DISTANCE_M = 1.5
    const val WP_ACCEPT_DISTANCE_M_HOLD_HEADING = 0.5// horizontal distance in meters
    const val WP_ACCEPT_ALTITUDE_M = 0.5    // vertical error in meters
    const val WP_ACCEPT_YAW_DEG = 4.0       // yaw error in degrees

    private fun distancePidKp(): Double = DroneControlProfiles.activeProfile().distanceKp
    private fun distancePidKi(): Double = DroneControlProfiles.activeProfile().distanceKi
    private fun distancePidKd(): Double = DroneControlProfiles.activeProfile().distanceKd
    private fun yawPidKp(): Double = DroneControlProfiles.activeProfile().yawKp
    private fun maxYawRateDegS(): Double = DroneControlProfiles.activeProfile().maxYawRateDegS
    private fun waypointPidOutputLimit(): Double = DroneControlProfiles.activeProfile().maxHorizontalSpeedMps
    private fun maxHorizontalAccelMps2(): Double = DroneControlProfiles.activeProfile().maxHorizontalAccelMps2

    // Listener interface so the UI can react to automatic activation
    interface ManualOverrideListener {
        fun onManualOverrideActivated()
    }
    var manualOverrideListener: ManualOverrideListener? = null

    /**
     * Called when RC stick input exceeds the deadzone during autonomous flight.
     * Activates the manual override latch and kills any running control loops.
     */
    fun activateManualOverride() {
        if (!isManualOverrideActive) {
            isManualOverrideActive = true
            cancelActiveControlLoop()
            virtualStickVM?.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() { }
                override fun onFailure(error: IDJIError) { }
            })
            setDroneStatus(DroneStatus.MANUAL_OVERRIDE)
            ToastUtils.showToast("⚠ MANUAL OVERRIDE ACTIVE — autonomous commands blocked")
            manualOverrideListener?.onManualOverrideActivated()
        }
    }

    /**
     * Called ONLY by the user pressing the deactivate button/checkbox.
     * Clears the manual override latch so autonomous commands work again.
     */
    fun deactivateManualOverride() {
        isManualOverrideActive = false
        setDroneStatus(DroneStatus.IDLE)
        ToastUtils.showToast("Manual override cleared — autonomous commands enabled")
    }

    /**
     * Check if an autonomous command should be allowed to execute.
     * Returns true if the command should be REJECTED (manual override is active).
     */
    fun shouldRejectAutonomousCommand(commandName: String = ""): Boolean {
        if (isManualOverrideActive) {
            val msg = if (commandName.isNotEmpty())
                "Command '$commandName' rejected — manual override active"
            else
                "Autonomous command rejected — manual override active"
            ToastUtils.showToast(msg)
            return true
        }
        return false
    }
    // ==================== End Manual Override ====================


    // ==================== Drone Status ====================
    /**
     * High-level operational state of the drone, derived from app-side command tracking.
     * The UI layer can also upgrade IDLE → HOVERING using FC telemetry (isFlying key).
     */
    enum class DroneStatus {
        IDLE, TAKING_OFF, HOVERING, NAVIGATING, LANDING, RETURNING_HOME, MANUAL_OVERRIDE, ABORTING
    }

    interface DroneStatusListener {
        fun onDroneStatusChanged(status: DroneStatus)
    }
    var droneStatusListener: DroneStatusListener? = null

    @Volatile
    var droneStatus: DroneStatus = DroneStatus.IDLE
        private set

    private val statusResetHandler = Handler(Looper.getMainLooper())

    private fun setDroneStatus(status: DroneStatus) {
        droneStatus = status
        droneStatusListener?.onDroneStatusChanged(status)
    }
    // ==================== End Drone Status ====================

    fun init(basicVM: BasicAircraftControlVM, stickVM: VirtualStickVM ) {
        basicAircraftControlVM = basicVM
        virtualStickVM = stickVM
    }

    fun destroy() {
        statusResetHandler.removeCallbacksAndMessages(null)
        manualOverrideListener = null
        droneStatusListener = null
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

    @Volatile private var _isWaypointReached = false
    // Monotonic id assigned to every waypoint navigation request (fresh start OR hot-swap).
    // Echoed in telemetry as "waypointSeq" so a ground station can tell whether a streamed
    // "waypointReached" refers to the target it just commanded or a stale latched value from
    // the previous one. Incremented from the HTTP server thread, read from the telemetry thread.
    private val _waypointSeq = java.util.concurrent.atomic.AtomicLong(0)
    @Volatile private var _isYawReached = false
    // Same role as _waypointSeq, for gotoYaw — echoed in telemetry as "yawSeq".
    private val _yawSeq = java.util.concurrent.atomic.AtomicLong(0)
    @Volatile private var _isAltitudeReached = false
    // Same role as _waypointSeq, for gotoAltitude — echoed in telemetry as "altitudeSeq".
    private val _altitudeSeq = java.util.concurrent.atomic.AtomicLong(0)
    @Volatile private var _isIntermediaryWaypointReached = false

    // Hot-swappable waypoint target for smooth PID transitions.
    // When a new waypoint arrives mid-flight, the target is swapped without restarting the loop.
    data class WaypointTarget(
        val latitude: Double,
        val longitude: Double,
        val altitude: Double,
        val yaw: Double,        // TRACK yaw: heading held while translating (bearing to the WP)
        val maxSpeed: Double,
        // FINAL yaw: heading the drone rotates to in place once it has arrived (Phase 3). Defaults
        // to the track yaw so callers that don't care about arrival heading keep the old behaviour.
        val finalYaw: Double = yaw
    )

    @Volatile
    private var activeWaypointTarget: WaypointTarget? = null

    // True only while the *waypoint* PID loop is the active control loop. controlLoopEnabled
    // is shared by every loop type (gotoYaw / gotoAltitude), but only the
    // waypoint loop consumes activeWaypointTarget — so a hot-swap must check this flag, otherwise
    // a waypoint command issued while another loop runs would be stashed and silently dropped.
    @Volatile
    private var activeLoopIsWaypoint = false

    // Which waypoint controller owns the running loop. flyToWaypointNoseForward and
    // flyToWaypointHoldHeading share activeWaypointTarget/activeLoopIsWaypoint but run
    // DIFFERENT motion laws and interpret WaypointTarget.yaw/finalYaw differently. A hot-swap is
    // only safe between calls of the SAME controller; swapping a target into the other controller's
    // runnable applies the wrong motion law. So the hot-swap gate must also match this mode,
    // otherwise it falls through to a cold restart with the correct runnable.
    private enum class WaypointMode { NOSE_FORWARD, HOLD_HEADING }
    @Volatile
    private var activeWaypointMode: WaypointMode? = null

    // Set when a target is hot-swapped into a running waypoint loop. The loop clears it on the
    // next tick by reset()-ing its PIDs, so the discontinuous jump in error doesn't kick.
    @Volatile
    private var waypointPidResetRequested = false

    // Control loop management - to prevent ghost waypoint navigation
    private var activeControlLoopHandler: Handler? = null
    private var activeControlLoopRunnable: Runnable? = null
    
    // Kill switch - when false, ALL control loops must stop immediately
    @Volatile
    private var controlLoopEnabled = false

    /**
     * True while a PID/virtual-stick control loop is actively running.
     * Used externally (e.g. VirtualStickVM) to gate the manual-override check so that
     * RC stick noise or drift doesn't spuriously activate override when the drone is idle.
     */
    val isAutonomousFlightActive: Boolean
        get() = controlLoopEnabled

    /**
     * Mirrors the FC isFlying telemetry key. Updated by the Activity via the
     * isFlyingKey listener so VirtualStickVM can gate manual-override detection:
     * sticks only latch override when the drone is actually airborne (prevents
     * ground-level calibration drift false-positives) or when an autonomous loop
     * is running (catches pilot intervention mid-mission).
     */
    @Volatile
    var isAirborne: Boolean = false

    // Unique ID for each control loop session - loops check this to ensure they're still valid
    @Volatile
    private var currentControlLoopId: Long = 0
    
    // Timestamp when control loop started - used to give virtual stick time to enable
    @Volatile
    private var controlLoopStartTime: Long = 0
    
    // Grace period (ms) to allow virtual stick to enable before checking its state
    private val VIRTUAL_STICK_ENABLE_GRACE_PERIOD_MS = 1000L

    /**
     * Cancel any active control loop (waypoint, gotoYaw, gotoAltitude, etc.)
     * This MUST be called before starting a new control loop or when disabling virtual sticks
     */
    fun cancelActiveControlLoop() {
        // Disable control loop and increment ID to invalidate any running loops
        controlLoopEnabled = false
        currentControlLoopId++
        activeWaypointTarget = null
        activeLoopIsWaypoint = false
        activeWaypointMode = null
        waypointPidResetRequested = false

        activeControlLoopRunnable?.let { runnable ->
            activeControlLoopHandler?.removeCallbacks(runnable)
        }
        activeControlLoopHandler?.removeCallbacksAndMessages(null)
        activeControlLoopHandler = null
        activeControlLoopRunnable = null
        // Reset stick to neutral position
        setStick(0F, 0F, 0F, 0F)
        // Reset navigation status — but don't overwrite TAKING_OFF, LANDING, RTH, MANUAL, ABORTING
        if (droneStatus == DroneStatus.NAVIGATING) setDroneStatus(DroneStatus.IDLE)
    }
    
    /**
     * Start a new control loop session - returns the loop ID that must be checked each iteration
     */
    private fun startNewControlLoopSession(): Long {
        cancelActiveControlLoop()  // Cancel any previous loop first
        controlLoopEnabled = true
        currentControlLoopId++
        controlLoopStartTime = System.currentTimeMillis()
        setDroneStatus(DroneStatus.NAVIGATING)
        return currentControlLoopId
    }
    
    /**
     * Check if the control loop with given ID should continue running.
     * Returns false if:
     * - The control loop was explicitly cancelled (controlLoopEnabled = false)
     * - A new control loop was started (loopId doesn't match)
     * - Manual override is active (pilot took control via RC sticks)
     * - The drone is no longer in virtual stick mode (user took manual control)
     */
    private fun shouldControlLoopContinue(loopId: Long): Boolean {
        // Check if loop was cancelled or a new one started
        if (!controlLoopEnabled || loopId != currentControlLoopId) {
            return false
        }

        // If manual override was triggered, stop immediately
        if (isManualOverrideActive) {
            controlLoopEnabled = false
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
        // If virtual stick gets disabled while a loop is running, kill the loop.
        val isVirtualStickEnabled = virtualStickVM?.currentVirtualStickStateInfo?.value?.state?.isVirtualStickEnable ?: false
        if (!isVirtualStickEnabled) {
            // Virtual stick was disabled externally.
            // NOTE: Do NOT call activateManualOverride() here — virtual stick can be disabled
            // by the system itself (e.g. disableVirtualStick() called by startTakeOff(), signal
            // loss recovery, FC safety checks) which would spuriously latch manual override and
            // block subsequent autonomous commands.  Real pilot RC-stick intervention is detected
            // in VirtualStickVM.tryUpdateVirtualStickByRc() while isAutonomousFlightActive is true.
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
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) { /* SDK may report "already enabled" — not a real error */ }
        })
    }

    fun disableVirtualStick() {
        // Cancel any active control loop first
        cancelActiveControlLoop()
        virtualStickVM?.disableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) { /* SDK may report "already disabled" — not a real error */ }
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
        setDroneStatus(DroneStatus.ABORTING)
        cancelActiveControlLoop()
        // ABORTING is a transient display state — return to IDLE after 2 s
        statusResetHandler.postDelayed({
            if (droneStatus == DroneStatus.ABORTING) setDroneStatus(DroneStatus.IDLE)
        }, 2_000L)
        
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
        setDroneStatus(DroneStatus.TAKING_OFF)
        basicAircraftControlVM?.startTakeOff(object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start takeOff onSuccess.")
                // Auto-reset after ~12 s; telemetry will upgrade IDLE → HOVERING if airborne
                statusResetHandler.postDelayed({
                    if (droneStatus == DroneStatus.TAKING_OFF) setDroneStatus(DroneStatus.IDLE)
                }, 12_000L)
            }
            override fun onFailure(error: IDJIError) {
                setDroneStatus(DroneStatus.IDLE)
                ToastUtils.showToast("start takeOff onFailure, $error")
            }
        })
    }

    fun startLanding() {
        setDroneStatus(DroneStatus.LANDING)
        statusResetHandler.postDelayed({
            if (droneStatus == DroneStatus.LANDING) setDroneStatus(DroneStatus.IDLE)
        }, 40_000L)
        basicAircraftControlVM?.startLanding(object : CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                ToastUtils.showToast("start landing onSuccess.")
            }
            override fun onFailure(error: IDJIError) {
                setDroneStatus(DroneStatus.IDLE)
                ToastUtils.showToast("start landing onFailure, $error")
            }
        })
    }

    fun startReturnToHome() {
        // CRITICAL: Disable virtual stick before RTH to prevent conflicts
        // Virtual stick mode can interfere with RTH causing erratic behaviorW
        setDroneStatus(DroneStatus.RETURNING_HOME)
        statusResetHandler.postDelayed({
            if (droneStatus == DroneStatus.RETURNING_HOME) setDroneStatus(DroneStatus.IDLE)
        }, 120_000L)
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

    /** Rotate to [targetYaw]. Returns the seq id published in telemetry as "yawSeq" so a caller
     *  can match a streamed "yawReached" to this command rather than a latched previous one. */
    fun gotoYaw(targetYaw: Double): Long {
        stopCurrentMission()
        // Start new control loop session
        val loopId = startNewControlLoopSession()

        val seq = _yawSeq.incrementAndGet()
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
                /* SDK may report "already enabled" — not a real error */
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
        return seq
    }

    /** Climb/descend to [targetAltitude]. Returns the seq id published in telemetry as
     *  "altitudeSeq" so a caller can match a streamed "altitudeReached" to this command. */
    fun gotoAltitude(targetAltitude: Double): Long {
        stopCurrentMission()
        // Start new control loop session
        val loopId = startNewControlLoopSession()

        val seq = _altitudeSeq.incrementAndGet()

        // Enable Virtual Stick and advanced mode
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) { /* SDK may report "already enabled" — not a real error */ }
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
                val Kp = 0.4 // Adjust this gain as needed

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
        return seq
    }

    /**
     * CONTRACT: hold-heading controller. The nose stays on the caller-supplied [targetYaw] for the
     * WHOLE flight — the to-waypoint vector is projected into the body frame, so the drone crabs
     * sideways/diagonally rather than turning to face where it is going. Use this when the payload
     * must keep looking at one bearing while repositioning (animal tracking, standoff observation).
     * Tighter arrival tolerance than [flyToWaypointNoseForward]: WP_ACCEPT_DISTANCE_M_HOLD_HEADING.
     * If you want the drone to fly nose-first along the leg instead, use [flyToWaypointNoseForward].
     *
     * Returns the monotonic sequence id assigned to this request; the same value is published in
     * telemetry as "waypointSeq" so a caller can confirm a streamed "waypointReached" belongs to
     * this target and not a latched previous one.
     */
    fun flyToWaypointHoldHeading(targetLatitude: Double, targetLongitude: Double, targetAlt: Double, targetYaw: Double, maxSpeed: Double): Long {
        val newTarget = WaypointTarget(targetLatitude, targetLongitude, targetAlt, targetYaw, maxSpeed)
        // New target → new id, and the reached latch drops to false until this target is reached.
        val seq = _waypointSeq.incrementAndGet()
        _isWaypointReached = false

        // If a *waypoint* PID loop is already running, just hot-swap the target.
        // The running loop reads activeWaypointTarget each tick and will smoothly steer to the new
        // waypoint without a cold restart. We must check activeLoopIsWaypoint, not just
        // controlLoopEnabled: that flag is shared by gotoYaw/gotoAltitude, none
        // of which consume activeWaypointTarget — so swapping into one of those would lose the
        // command. The reset request clears the PID derivative/integral kick from the error jump.
        // Also require the SAME controller mode: if the running loop is flyToWaypointNoseForward's
        // runnable, its motion law/yaw interpretation differ from this precise one, so we must NOT
        // hot-swap into it — fall through to a cold restart with the precise runnable instead.
        if (controlLoopEnabled && activeLoopIsWaypoint && activeWaypointMode == WaypointMode.HOLD_HEADING) {
            activeWaypointTarget = newTarget
            waypointPidResetRequested = true
            return seq
        }

        // No active waypoint loop (or a different controller is running) — start a fresh one
        // (this also cancels any other active loop). Set target AFTER startNewControlLoopSession()
        // because it calls cancelActiveControlLoop() which clears activeWaypointTarget.
        stopCurrentMission()
        val loopId = startNewControlLoopSession()
        activeWaypointTarget = newTarget
        activeLoopIsWaypoint = true
        activeWaypointMode = WaypointMode.HOLD_HEADING

        val updateInterval = 100.0  // Nominal update period (ms); real dt is measured each tick.
        val maxYawRate = maxYawRateDegS() // degrees per second, from the active drone profile
        var lastCommandedSpeed = 0.0
        var lastTickMs = 0L  // SystemClock.elapsedRealtime() of the previous tick, 0 = first tick

        virtualStickVM?.enableVirtualStickAdvancedMode()
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                /* SDK may report "already enabled" — not a real error */
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()

        // PID gains are all selected from the connected aircraft profile at runtime.
        val distancePID = PID(distancePidKp(), distancePidKi(), distancePidKd(), updateInterval/1000, 0.0 to waypointPidOutputLimit())
        val yawPID = PID(yawPidKp(), 0.0000, 0.00, updateInterval/1000, -maxYawRate to maxYawRate)

        val controlLoop = Handler(Looper.getMainLooper())
        virtualStickVM?.enableVirtualStickAdvancedMode()

        // Cooldown: after reaching a waypoint, keep PID loop alive for this long
        // to allow the bridge to hot-swap the next target without a cold restart.
        val holdCooldownMs = 200L
        var reachedAtMs = 0L  // SystemClock.elapsedRealtime() when waypoint was first reached, 0 = not reached

        val runnable = object : Runnable {
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }

                // Read the current target (may have been hot-swapped by a new call)
                val target = activeWaypointTarget
                if (target == null) {
                    setStick(0F, 0F, 0F, 0F)
                    controlLoopEnabled = false
                    disableVirtualStick()
                    return
                }

                // Measure the real timestep instead of assuming updateInterval. The loop runs on
                // the main Looper, whose cadence drifts under load; clamp so a stalled thread can't
                // inject a huge dt spike into the PID derivative/integral or the accel limiter.
                val nowMs = android.os.SystemClock.elapsedRealtime()
                val dtSec = if (lastTickMs == 0L) updateInterval / 1000.0
                            else ((nowMs - lastTickMs) / 1000.0).coerceIn(0.02, 0.5)
                lastTickMs = nowMs

                // A hot-swapped target is a discontinuous setpoint — clear PID history once so the
                // jump in distance/yaw error doesn't produce an integral/derivative kick.
                if (waypointPidResetRequested) {
                    waypointPidResetRequested = false
                    distancePID.reset()
                    yawPID.reset()
                }

                val currentPosition = getLocation3D()
                val currentYaw = getHeading()

                val distance = calculateDistance(target.latitude, target.longitude, currentPosition.latitude, currentPosition.longitude)
                val pidSpeed = distancePID.update(distance, dtSec).coerceAtMost(target.maxSpeed)
                val maxSpeedStep = maxHorizontalAccelMps2() * dtSec
                val targetSpeed = pidSpeed.coerceAtMost(lastCommandedSpeed + maxSpeedStep)
                lastCommandedSpeed = targetSpeed
                val movementDirection = calculateBearing(currentPosition.latitude, currentPosition.longitude, target.latitude, target.longitude).toDouble()

                val yawError = normalizeAngle(target.yaw - currentYaw)
                val angularVelocity = yawPID.update(yawError, dtSec)

                val movementDirectionRelative = normalizeAngle(movementDirection - currentYaw) // Relative to the drone's heading
                val forwardSpeed = targetSpeed * cos(Math.toRadians(movementDirectionRelative))
                val lateralSpeed = targetSpeed * sin(Math.toRadians(movementDirectionRelative))

                val altError = target.altitude - currentPosition.altitude

                if (distance < WP_ACCEPT_DISTANCE_M_HOLD_HEADING && abs(yawError) < WP_ACCEPT_YAW_DEG && abs(altError) < WP_ACCEPT_ALTITUDE_M) {
                    val now = android.os.SystemClock.elapsedRealtime()
                    if (!_isWaypointReached) {
                        _isWaypointReached = true
                        reachedAtMs = now
                    }
                    // Cooldown expired — no new waypoint arrived, stop cleanly
                    if (now - reachedAtMs >= holdCooldownMs) {
                        setStick(0F, 0F, 0F, 0F)
                        activeWaypointTarget = null
                        controlLoopEnabled = false
                        disableVirtualStick()
                        return
                    }
                } else {
                    // Moved outside acceptance (e.g. GPS drift or new target) — reset cooldown
                    reachedAtMs = 0L
                }

                // DJI SDK V5 quirk: in BODY frame, the SDK's "pitch" field actually controls
                // lateral (left/right) movement and "roll" controls forward/backward. This is
                // the inverse of what the field names suggest. Confirmed empirically.
                val flightControlParam = VirtualStickFlightControlParam().apply {
                    this.pitch = lateralSpeed
                    this.roll = forwardSpeed
                    this.yaw = angularVelocity
                    this.verticalThrottle = target.altitude
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
        return seq
    }

    /**
     * CONTRACT: nose-follows-path controller with a final-heading phase. Three phases:
     *   Phase 1 ALIGN  — rotate in place to the TRACK heading = bearing(current -> WP).
     *   Phase 2 NAV    — translate to the WP, nose held on the track heading (cross-track owns lateral).
     *   Phase 3 FINAL  — once arrived (position + altitude), rotate in place to the user-requested
     *                    [targetYaw], holding position. Only then is the waypoint latched as reached.
     * So during travel the drone faces its direction of motion (required by the Phase-2 motion law,
     * which dead-reckons forwardSpeed along body-X), and [targetYaw] sets the heading the drone ends
     * up facing AT the waypoint. If you need the nose pointed at [targetYaw] while translating, use
     * [flyToWaypointHoldHeading] instead, which projects the to-waypoint vector into body frame.
     */
    fun flyToWaypointNoseForward(targetLatitude: Double, targetLongitude: Double, targetAlt: Double, targetYaw: Double, maxSpeed: Double): Long {
        // Track yaw = bearing(current position -> waypoint): the heading held during Phase 1/2.
        // The caller-supplied targetYaw becomes the Phase-3 final heading (finalYaw). Recomputed here
        // so both the cold-start and hot-swap paths below anchor the nose on the new leg.
        val startPos = getLocation3D()
        val trackYaw = calculateBearing(startPos.latitude, startPos.longitude, targetLatitude, targetLongitude).toDouble()
        val newTarget = WaypointTarget(targetLatitude, targetLongitude, targetAlt, trackYaw, maxSpeed, finalYaw = targetYaw)
        // New target → new id, and the reached latch drops to false until this target is reached.
        val seq = _waypointSeq.incrementAndGet()
        _isWaypointReached = false

        // If a *waypoint* PID loop is already running, just hot-swap the target.
        // The running loop reads activeWaypointTarget each tick and will smoothly steer to the new
        // waypoint without a cold restart. We must check activeLoopIsWaypoint, not just
        // controlLoopEnabled: that flag is shared by gotoYaw/gotoAltitude, none
        // of which consume activeWaypointTarget — so swapping into one of those would lose the
        // command. The reset request clears the PID derivative/integral kick from the error jump.
        // Also require the SAME controller mode: if the running loop is the precise runnable, its
        // motion law/yaw interpretation differ from this one, so we must NOT hot-swap into it —
        // fall through to a cold restart with this runnable instead.
        if (controlLoopEnabled && activeLoopIsWaypoint && activeWaypointMode == WaypointMode.NOSE_FORWARD) {
            activeWaypointTarget = newTarget
            waypointPidResetRequested = true
            return seq
        }

        // No active waypoint loop (or a different controller is running) — start a fresh one
        // (this also cancels any other active loop). Set target AFTER startNewControlLoopSession()
        // because it calls cancelActiveControlLoop() which clears activeWaypointTarget.
        stopCurrentMission()
        val loopId = startNewControlLoopSession()
        activeWaypointTarget = newTarget
        activeLoopIsWaypoint = true
        activeWaypointMode = WaypointMode.NOSE_FORWARD

        val updateInterval = 100.0  // PID/setpoint recompute period (ms); real dt measured each control tick.
        val sendIntervalMs = 100L    // Virtual-stick resend period (ms) = 10 Hz, decoupled from the 1 Hz
                                     // control update. The SDK watchdog zeros the sticks if no fresh
                                     // command arrives within a few hundred ms, so we re-send the latest
                                     // computed param at 10 Hz to keep pitch/roll/yaw velocity continuous
                                     // between PID updates — this kills the once-per-second stutter.
        val maxYawRate = maxYawRateDegS() // degrees per second, from the active drone profile
        var lastCommandedSpeed = 0.0
        var lastControlMs = 0L  // elapsedRealtime() of the last PID/setpoint update, 0 = first tick
        var lastParam: VirtualStickFlightControlParam? = null  // latest computed command, resent at 10 Hz

        virtualStickVM?.enableVirtualStickAdvancedMode()
        // NOTE: Use VM directly, not enableVirtualStick() which would cancel the loop we just started
        virtualStickVM?.enableVirtualStick(object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() { }
            override fun onFailure(error: IDJIError) {
                /* SDK may report "already enabled" — not a real error */
            }
        })
        virtualStickVM?.enableVirtualStickAdvancedMode()

        // --- Cross-track lateral control state (replaces speed-scaled bearing steering) ---
        // Roll-oscillation root cause: the old law lateralSpeed = targetSpeed*sin(bearing-yaw)
        // scaled the cross-track correction by full forward speed AND by ~1/distance near the WP,
        // so it blew up into a roll limit cycle at speed. We instead steer on the signed
        // perpendicular distance to the straight A->B line: well-conditioned near the endpoint and
        // with a gain independent of speed and distance.
        var lineLat0 = 0.0            // A: track origin, captured at cold start / after a hot-swap
        var lineLon0 = 0.0
        var lineValid = false
        var crossTrackFilt = 0.0      // low-pass of signed cross-track offset (m); kills stepped-GPS jitter
        var crossTrackInit = false
        var lastLateralSpeed = 0.0    // for the lateral slew limit
        var yawAligned = false        // Phase 1->2: false => rotate to track heading; true => translate
        var positionReached = false   // Phase 2->3: true once within WP distance+altitude; Phase 3 then
                                      // rotates in place to target.finalYaw before latching reached.
        val crossTrackKp = 0.5        // m/s of lateral correction per m of offset
        val maxLateralSpeed = 3.0     // hard cap on lateral correction (m/s)
        val crossTrackLpfAlpha = 0.3  // 0..1 low-pass weight; lower = smoother

        // All distance/yaw gains and the max horizontal accel come from the active aircraft profile,
        // same as flyToWaypointHoldHeading. (This endpoint exists for the align-then-translate
        // behaviour, not for live gain sweeping.)
        val distancePID = PID(distancePidKp(), distancePidKi(), distancePidKd(), updateInterval/1000, 0.0 to waypointPidOutputLimit())
        val yawPID = PID(yawPidKp(), 0.0000, 0.00, updateInterval/1000, -maxYawRate to maxYawRate)

        val controlLoop = Handler(Looper.getMainLooper())
        virtualStickVM?.enableVirtualStickAdvancedMode()

        // Cooldown: after reaching a waypoint, keep PID loop alive for this long
        // to allow the bridge to hot-swap the next target without a cold restart.
        val holdCooldownMs = 200L
        var reachedAtMs = 0L  // SystemClock.elapsedRealtime() when waypoint was first reached, 0 = not reached

        val runnable = object : Runnable {
            
            override fun run() {
                // CHECK IF WE SHOULD STILL BE RUNNING
                if (!shouldControlLoopContinue(loopId)) {
                    setStick(0F, 0F, 0F, 0F)
                    return
                }

                // Read the current target (may have been hot-swapped by a new call)
                val target = activeWaypointTarget
                if (target == null) {
                    setStick(0F, 0F, 0F, 0F)
                    controlLoopEnabled = false
                    disableVirtualStick()
                    return
                }

                val nowMs = android.os.SystemClock.elapsedRealtime()
                // Between PID updates: just re-send the latest command so the SDK watchdog never
                // zeros the sticks. This is what makes the motion continuous instead of stuttering
                // once per second — the setpoint only changes at 1 Hz but the drone keeps the
                // commanded velocity the whole time.
                if (lastControlMs != 0L && (nowMs - lastControlMs) < updateInterval.toLong()) {
                    lastParam?.let { virtualStickVM?.sendVirtualStickAdvancedParam(it) }
                    controlLoop.postDelayed(this, sendIntervalMs)
                    return
                }

                // Control tick: measure real elapsed time since the last PID update. Clamp so a
                // stalled main Looper can't inject a huge dt spike into the PID or accel limiter.
                val dtSec = if (lastControlMs == 0L) updateInterval / 1000.0
                else ((nowMs - lastControlMs) / 1000.0).coerceIn(0.02, 2.0)
                lastControlMs = nowMs

                // A hot-swapped target is a discontinuous setpoint — clear PID history once so the
                // jump in distance/yaw error doesn't produce an integral/derivative kick. Also drop
                // the cross-track line + lateral state so they re-anchor to the new leg.
                if (waypointPidResetRequested) {
                    waypointPidResetRequested = false
                    distancePID.reset()
                    yawPID.reset()
                    lineValid = false
                    crossTrackInit = false
                    lastLateralSpeed = 0.0
                    yawAligned = false
                    positionReached = false
                }

                val currentPosition = getLocation3D()
                val currentYaw = getHeading()

                // Phase 1 (ALIGN): rotate yaw in place toward target.yaw, no translation, holding
                // altitude. Only when the heading is within tolerance do we switch to Phase 2 (NAV).
                // We early-return here so the distance/cross-track loops don't tick during rotation:
                // distancePID stays un-wound and the A->B line anchors where translation begins.
                val yawError = normalizeAngle(target.yaw - currentYaw)
                val angularVelocity = yawPID.update(yawError, dtSec)
                if (!yawAligned) {
                    if (abs(yawError) < WP_ACCEPT_YAW_DEG) {
                        yawAligned = true
                    } else {
                        lastParam = VirtualStickFlightControlParam().apply {
                            this.pitch = 0.0
                            this.roll = 0.0
                            this.yaw = angularVelocity
                            this.verticalThrottle = target.altitude
                            this.verticalControlMode = VerticalControlMode.POSITION
                            this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                            this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                            this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                        }
                        lastParam?.let { virtualStickVM?.sendVirtualStickAdvancedParam(it) }
                        controlLoop.postDelayed(this, sendIntervalMs)
                        return
                    }
                }

                // Anchor the straight-line track origin A on the first tick of this leg.
                if (!lineValid) {
                    lineLat0 = currentPosition.latitude
                    lineLon0 = currentPosition.longitude
                    lineValid = true
                }

                val distance = calculateDistance(target.latitude, target.longitude, currentPosition.latitude, currentPosition.longitude)
                val altErrorNav = target.altitude - currentPosition.altitude

                // Phase 2 -> 3 transition: latch positionReached once inside the WP position + altitude
                // tolerance. From then on we stop translating and run Phase 3 (FINAL ALIGN).
                if (!positionReached && distance < WP_ACCEPT_DISTANCE_M && abs(altErrorNav) < WP_ACCEPT_ALTITUDE_M) {
                    positionReached = true
                }

                // Phase 3 (FINAL ALIGN): rotate in place to the user-requested arrival heading
                // (target.finalYaw), holding position and altitude. The waypoint is only latched as
                // reached once that heading is within tolerance; until then the cooldown stays unarmed.
                if (positionReached) {
                    val finalYawError = normalizeAngle(target.finalYaw - currentYaw)
                    val finalAngularVelocity = yawPID.update(finalYawError, dtSec)
                    val now = android.os.SystemClock.elapsedRealtime()
                    if (abs(finalYawError) < WP_ACCEPT_YAW_DEG) {
                        if (!_isWaypointReached) {
                            _isWaypointReached = true
                            reachedAtMs = now
                        }
                        // Cooldown expired — no new waypoint hot-swapped in, stop cleanly.
                        if (now - reachedAtMs >= holdCooldownMs) {
                            setStick(0F, 0F, 0F, 0F)
                            activeWaypointTarget = null
                            controlLoopEnabled = false
                            disableVirtualStick()
                            return
                        }
                    } else {
                        reachedAtMs = 0L  // still rotating to final heading — keep the cooldown unarmed
                    }
                    lastParam = VirtualStickFlightControlParam().apply {
                        this.pitch = 0.0
                        this.roll = 0.0
                        this.yaw = finalAngularVelocity
                        this.verticalThrottle = target.altitude
                        this.verticalControlMode = VerticalControlMode.POSITION
                        this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                        this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                        this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                    }
                    lastParam?.let { virtualStickVM?.sendVirtualStickAdvancedParam(it) }
                    controlLoop.postDelayed(this, sendIntervalMs)
                    return
                }

                // Forward speed: distance-PID, clamped to maxSpeed, a kinematic decel cap so the
                // drone can always brake within the remaining distance (v <= sqrt(2*a*d)), then the
                // existing accel slew limit. The decel cap replaces the implicit, Kp-defined braking
                // zone that used to slam the brakes / overshoot near the WP.
                val brakeDist = max(0.0, distance - WP_ACCEPT_DISTANCE_M)
                val decelCap = sqrt(2.0 * maxHorizontalAccelMps2() * brakeDist)
                val pidSpeed = distancePID.update(distance, dtSec)
                    .coerceAtMost(target.maxSpeed)
                    .coerceAtMost(decelCap)
                val maxSpeedStep = maxHorizontalAccelMps2() * dtSec
                val targetSpeed = pidSpeed.coerceAtMost(lastCommandedSpeed + maxSpeedStep)
                lastCommandedSpeed = targetSpeed

                // --- Cross-track lateral correction (decoupled from forward speed) ---
                // Signed perpendicular distance from the drone to the A->B line, in meters.
                // x = north, y = east. cross = ((B-A) x (P-A)).z / |B-A|; positive => right of track.
                val mPerDegLat = 111320.0
                val mPerDegLon = 111320.0 * cos(Math.toRadians(currentPosition.latitude))
                val bn = (target.latitude - lineLat0) * mPerDegLat
                val be = (target.longitude - lineLon0) * mPerDegLon
                val pn = (currentPosition.latitude - lineLat0) * mPerDegLat
                val pe = (currentPosition.longitude - lineLon0) * mPerDegLon
                val segLen = hypot(bn, be)
                val crossTrackRaw = if (segLen > 0.1) (bn * pe - be * pn) / segLen else 0.0
                crossTrackFilt = if (!crossTrackInit) crossTrackRaw
                                 else crossTrackFilt + crossTrackLpfAlpha * (crossTrackRaw - crossTrackFilt)
                crossTrackInit = true

                // Push back toward the line (positive cross = right of track -> steer left, i.e.
                // negative lateral). Constant gain (independent of speed and 1/distance) => no roll
                // limit cycle, and the line offset stays well-conditioned right up to the endpoint.
                val lateralDesired = (-crossTrackKp * crossTrackFilt).coerceIn(-maxLateralSpeed, maxLateralSpeed)
                // Slew-limit lateral like forward so it can't jump full range on one stepped GPS tick.
                val maxLatStep = maxHorizontalAccelMps2() * dtSec
                val lateralSpeed = lateralDesired.coerceIn(lastLateralSpeed - maxLatStep, lastLateralSpeed + maxLatStep)
                lastLateralSpeed = lateralSpeed

                // Forward is along body-X; the yaw loop holds the nose on the track heading so
                // body-forward stays aligned with A->B and the cross-track loop owns lateral. Sign
                // by along-track remaining so an overshoot past B reverses instead of running away.
                val alongRemaining = if (segLen > 0.1) segLen - (pn * bn + pe * be) / segLen else 0.0
                val forwardSpeed = if (alongRemaining >= 0.0) targetSpeed else -targetSpeed

                // Arrival (position + final-yaw) is handled by the Phase 3 block above, which
                // early-returns. Reaching here means we are still translating (Phase 2).

                // DJI SDK V5 quirk: in BODY frame, the SDK's "pitch" field actually controls
                // lateral (left/right) movement and "roll" controls forward/backward. This is
                // the inverse of what the field names suggest. Confirmed empirically.
                lastParam = VirtualStickFlightControlParam().apply {
                    this.pitch = lateralSpeed
                    this.roll = forwardSpeed
                    this.yaw = 0.0   // no yaw command during translation; heading set once in Phase 1
                    this.verticalThrottle = target.altitude
                    this.verticalControlMode = VerticalControlMode.POSITION
                    this.rollPitchControlMode = RollPitchControlMode.VELOCITY
                    this.yawControlMode = YawControlMode.ANGULAR_VELOCITY
                    this.rollPitchCoordinateSystem = FlightCoordinateSystem.BODY
                }

                lastParam?.let { virtualStickVM?.sendVirtualStickAdvancedParam(it) }
                controlLoop.postDelayed(this, sendIntervalMs)
            }
        }

        // Store references to allow cancellation
        activeControlLoopHandler = controlLoop
        activeControlLoopRunnable = runnable
        controlLoop.post(runnable)
        return seq
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

    // Id of the most recently accepted waypoint request. Pair this with isWaypointReached()
    // in telemetry so a client can match "reached" to a specific commanded target.
    fun getWaypointSeq(): Long {
        return _waypointSeq.get()
    }

    // Id of the most recently accepted gotoYaw request — pair with isYawReached().
    fun getYawSeq(): Long {
        return _yawSeq.get()
    }

    // Id of the most recently accepted gotoAltitude request — pair with isAltitudeReached().
    fun getAltitudeSeq(): Long {
        return _altitudeSeq.get()
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
