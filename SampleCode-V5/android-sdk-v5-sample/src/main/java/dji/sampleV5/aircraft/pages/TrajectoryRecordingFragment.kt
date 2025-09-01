package dji.sampleV5.aircraft.pages

import android.annotation.SuppressLint
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.annotation.RequiresApi
import com.dji.wpmzsdk.common.data.Template
import com.dji.wpmzsdk.manager.WPMZManager
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.databinding.FragTrajectoryRecordingPageBinding
import dji.sampleV5.aircraft.models.MissionGlobalModel
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sampleV5.aircraft.utils.wpml.WaypointInfoModel
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.common.LocationCoordinate2D
import dji.sdk.wpmz.value.mission.*
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.waypoint3.WaypointActionListener
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.utils.common.ContextUtil
import java.io.File
import java.io.FileOutputStream
import java.io.InputStream
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.TimeZone
import java.util.zip.ZipEntry
import java.util.zip.ZipFile
import kotlin.math.cos
import kotlin.math.sin

/**
 * Trajectory Recording and Replay Fragment
 * Generates a circular trajectory and replays it via Waypoint V3 (WPMZ)
 *
 * KMZ is saved to app-owned dir:
 *   /storage/emulated/0/Android/data/<pkg>/files/kmz/<name>.kmz
 */
class TrajectoryRecordingFragment : DJIFragment() {

    private var binding: FragTrajectoryRecordingPageBinding? = null

    // Recording state
    private var recordedWaypoints: ArrayList<WaypointInfoModel> = ArrayList()
    private var currentTrajectoryName = ""

    // Mission management
    private val missionGlobalModel = MissionGlobalModel()
    private var curMissionPath = ""
    private var currentMissionNameNoExt = ""   // <-- IMPORTANT: used for start/stop

    // File settings
    private val TRAJECTORY_FILE_TAG = ".kmz"

    // Debug
    private val logTag = "WPV3-Trajectory"
    private val debugSb = StringBuilder()

    // App-owned external files directory for KMZ output
    private val kmzDir: String by lazy {
        val ctx = ContextUtil.getContext()
        val base = ctx.getExternalFilesDir(null) // e.g. .../Android/data/<pkg>/files
        val dir = File(base, "kmz").apply { mkdirs() }
        dir.absolutePath + File.separator
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?,
    ): View? {
        binding = FragTrajectoryRecordingPageBinding.inflate(inflater, container, false)
        return binding?.root
    }

    @RequiresApi(Build.VERSION_CODES.O)
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initView()
        initData()
        WPMZManager.getInstance().init(ContextUtil.getContext())
    }

    @RequiresApi(Build.VERSION_CODES.O)
    private fun initView() {
        binding?.apply {
            // Recording controls
            btnStartRecording.setOnClickListener { startRecording() }
            btnStopRecording.setOnClickListener { stopRecording() }
            btnCaptureWaypoint.setOnClickListener { captureWaypoint() }
            btnSaveTrajectory.setOnClickListener { saveTrajectory() }

            // Replay controls
            btnLoadTrajectory.setOnClickListener { loadTrajectory() }
            btnStartReplay.setOnClickListener { startReplay() }
            btnPauseReplay.setOnClickListener { pauseReplay() }
            btnStopReplay.setOnClickListener { stopReplay() }

            // Initial UI state
            false.updateUIState(false)
        }
    }

    private fun initData() {
        // Explicit defaults to satisfy firmware expectations
        missionGlobalModel.globalSpeed = 10.0
        missionGlobalModel.finishAction = WaylineFinishedAction.GO_HOME
        missionGlobalModel.lostAction = WaylineExitOnRCLostAction.GO_BACK

        observeAircraftLocation()
        addWaylineExecutingInfoListener()
        addWaypointActionListener()
    }

    private fun startRecording() {
        val aircraftLocationKey = KeyTools.createKey(FlightControllerKey.KeyAircraftLocation)
        val aircraftLocation = KeyManager.getInstance().getValue(aircraftLocationKey)

        if (aircraftLocation == null) {
            ToastUtils.showToast("Unable to get aircraft location")
            return
        }

        val altitude = KeyManager.getInstance().getValue(
            KeyTools.createKey(FlightControllerKey.KeyAltitude)
        ) ?: 0f

        // Generate circular trajectory automatically
        recordedWaypoints.clear()
        currentTrajectoryName = generateTrajectoryName()

        generateCircularTrajectory(aircraftLocation, altitude.toDouble())

        ToastUtils.showToast("Generated circular trajectory: $currentTrajectoryName with ${recordedWaypoints.size} waypoints")
        false.updateUIState(false)

        binding?.tvWaypointCount?.text = getString(R.string.waypoints_count, recordedWaypoints.size)
        binding?.btnSaveTrajectory?.isEnabled = recordedWaypoints.isNotEmpty()
    }

    private fun stopRecording() {
        ToastUtils.showToast("Circular trajectory generation completed")
    }

    private fun captureWaypoint() {
        ToastUtils.showToast("Waypoints are automatically generated in circular pattern")
    }

    private fun saveTrajectory() {
        if (recordedWaypoints.isEmpty()) {
            ToastUtils.showToast("No waypoints to save")
            return
        }

        val kmzOutPath = kmzDir + currentTrajectoryName + TRAJECTORY_FILE_TAG
        val waylineMission: WaylineMission = createWaylineMission()
        val missionConfig: WaylineMissionConfig = createMissionConfig(missionGlobalModel)
        val template: Template = createTemplate(recordedWaypoints)

        WPMZManager.getInstance().generateKMZFile(kmzOutPath, waylineMission, missionConfig, template)

        curMissionPath = kmzOutPath
        currentMissionNameNoExt = File(kmzOutPath).nameWithoutExtension

        ToastUtils.showToast("Trajectory saved: $kmzOutPath")
        binding?.tvCurrentTrajectory?.text = getString(R.string.current_trajectory, currentTrajectoryName)
        false.updateUIState(true)

        // --- Debug: immediately inspect the KMZ and write a log
        debugSb.clear()
        debugSb.appendLine("=== WaypointV3 Debug Log ===")
        debugSb.appendLine("KMZ: $curMissionPath")
        debugSb.appendLine("Time: ${nowIso()}")
        debugSb.appendLine("============================")
        dumpKmzQuickCheck(curMissionPath, debugSb)
        appendLogWaylineIds(curMissionPath, debugSb)
        appendLogSdkParse(curMissionPath, debugSb)
        saveDebugLog(curMissionPath, debugSb.toString())
    }

    private fun loadTrajectory() {
        if (curMissionPath.isEmpty()) {
            ToastUtils.showToast("No trajectory saved yet")
            return
        }

        val waypointFile = File(curMissionPath)
        if (!waypointFile.exists() || !waypointFile.canRead()) {
            ToastUtils.showToast("KMZ not readable at $curMissionPath")
            return
        }

        logI("loadTrajectory() pushing KMZ: $curMissionPath")
        dumpKmzQuickCheck(curMissionPath, debugSb)
        saveDebugLog(curMissionPath, debugSb.toString())

        WaypointMissionManager.getInstance().pushKMZFileToAircraft(curMissionPath, object :
            CommonCallbacks.CompletionCallbackWithProgress<Double> {
            override fun onProgressUpdate(progress: Double) {
                logI("pushKMZ progress=${(progress * 10000).toInt()},0%") // match prior log format
            }
            override fun onSuccess() {
                logI("pushKMZ success")
                ToastUtils.showToast("Trajectory loaded successfully")
                binding?.tvMissionStatus?.text = getString(R.string.status_loaded)
                false.updateUIState(true)
                saveDebugLog(curMissionPath, debugSb.toString())
            }
            override fun onFailure(error: IDJIError) {
                logE("pushKMZ failed: ${error.errorCode()} ${error.description()}")
                ToastUtils.showToast("Load failed: ${error.description()}")
                saveDebugLog(curMissionPath, debugSb.toString())
            }
        })
    }

    @RequiresApi(Build.VERSION_CODES.O)
    private fun startReplay() {
        if (currentMissionNameNoExt.isEmpty()) {
            ToastUtils.showToast("No mission file loaded")
            logE("startReplay aborted: currentMissionNameNoExt is empty")
            return
        }
        if (curMissionPath.isEmpty()) {
            ToastUtils.showToast("No local KMZ path")
            logE("startReplay aborted: curMissionPath is empty")
            return
        }

        // Try to parse available wayline IDs; some SDK builds hide fields
        val idsFromFile = extractWaylineIdsFromKmz(curMissionPath)
        logI("Wayline IDs from waylines.wpml: $idsFromFile")

        // Also try via SDK parse (but its getWaylineID may be hidden)
        val idsFromSdk = getWaylineIdsViaSdk(curMissionPath)
        if (idsFromSdk.isNotEmpty()) {
            logI("Wayline IDs via SDK reflection: $idsFromSdk")
        }

        val idList = when {
            idsFromFile.isNotEmpty() -> idsFromFile
            idsFromSdk.isNotEmpty() -> idsFromSdk
            else -> arrayListOf(0)
        }
        logI("Wayline IDs to start: $idList (mission=$currentMissionNameNoExt)")

        WaypointMissionManager.getInstance().startMission(
            currentMissionNameNoExt,   // <-- name WITHOUT .kmz
            idList,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("Trajectory replay started")
                    binding?.tvMissionStatus?.text = getString(R.string.status_replaying)
                    saveDebugLog(curMissionPath, debugSb.toString())
                }
                override fun onFailure(error: IDJIError) {
                    val msg = "startMission failed code=${error.errorCode()} desc=${error.description()} ids=$idList"
                    logE(msg)
                    ToastUtils.showToast("Start replay failed: ${error.description()}")
                    saveDebugLog(curMissionPath, debugSb.toString())
                }
            }
        )
    }

    private fun pauseReplay() {
        WaypointMissionManager.getInstance().pauseMission(object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("Trajectory replay paused")
                binding?.tvMissionStatus?.text = getString(R.string.status_paused)
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("Pause failed: ${error.description()}")
            }
        })
    }

    private fun stopReplay() {
        WaypointMissionManager.getInstance().stopMission(currentMissionNameNoExt, object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                ToastUtils.showToast("Trajectory replay stopped")
                binding?.tvMissionStatus?.text = getString(R.string.status_stopped)
                false.updateUIState(true)
            }
            override fun onFailure(error: IDJIError) {
                ToastUtils.showToast("Stop failed: ${error.description()}")
            }
        })
    }

    /**
     * Build one waypoint. NOTE: WaylineWaypoint uses height/ellipsoidHeight, not "altitude".
     */
    private fun createWaypointFromLocation(
        location: LocationCoordinate2D,
        heightMeters: Double,
        index: Int
    ): WaypointInfoModel {
        val waypointInfo = WaypointInfoModel()
        val waypoint = WaylineWaypoint()

        // Location (2D)
        val coordinate2D = WaylineLocationCoordinate2D().apply {
            latitude = location.latitude
            longitude = location.longitude
        }
        waypoint.location = coordinate2D

        // Index
        waypoint.waypointIndex = index

        // Heights
        waypoint.height = heightMeters
        waypoint.ellipsoidHeight = heightMeters
        waypoint.useGlobalFlightHeight = false

        // Speed / turn params come from template
        waypoint.useGlobalAutoFlightSpeed = true
        waypoint.useGlobalTurnParam = true

        // Yaw
        val yawParam = WaylineWaypointYawParam().apply {
            yawMode = WaylineWaypointYawMode.FOLLOW_WAYLINE
            yawPathMode = WaylineWaypointYawPathMode.FOLLOW_BAD_ARC
            poiLocation = WaylineLocationCoordinate3D(location.latitude, location.longitude, heightMeters)
        }
        waypoint.yawParam = yawParam
        waypoint.useGlobalYawParam = false
        waypoint.isWaylineWaypointYawParamSet = true

        // Gimbal (required because template uses USE_POINT_SETTING)
        val gimbalParam = WaylineWaypointGimbalHeadingParam().apply {
            headingMode = WaylineWaypointGimbalHeadingMode.find(0)
            pitchAngle = 0.0
        }
        waypoint.gimbalHeadingParam = gimbalParam
        waypoint.isWaylineWaypointGimbalHeadingParamSet = true
        waypoint.useGlobalGimbalHeadingParam = false

        waypointInfo.waylineWaypoint = waypoint
        waypointInfo.actionInfos = ArrayList() // no actions for this demo
        return waypointInfo
    }

    // === Minimal in-fragment builders (bypass KMZTestUtil) ===
    private fun createWaylineMission(): WaylineMission {
        val m = WaylineMission()
        val now = System.currentTimeMillis().toDouble()
        m.createTime = now
        m.updateTime = now
        return m
    }

    private fun createMissionConfig(model: MissionGlobalModel): WaylineMissionConfig {
        val c = WaylineMissionConfig()
        c.flyToWaylineMode = WaylineFlyToWaylineMode.SAFELY
        c.finishAction = model.finishAction
        c.droneInfo = WaylineDroneInfo()
        c.securityTakeOffHeight = 20.0
        c.isSecurityTakeOffHeightSet = true
        c.exitOnRCLostBehavior = WaylineExitOnRCLostBehavior.EXCUTE_RC_LOST_ACTION
        c.exitOnRCLostType = model.lostAction
        c.globalTransitionalSpeed = 10.0
        c.payloadInfo = ArrayList<WaylinePayloadInfo>()
        return c
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
        t.useGlobalTransitionalSpeed = true   // <-- was isUseGlobalTransitionalSpeed = true
        t.autoFlightSpeed = 5.0
        t.payloadParam = ArrayList()
        return t
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
        info.globalTurnMode = WaylineWaypointTurnMode.TO_POINT_AND_STOP_WITH_DISCONTINUITY_CURVATURE
        info.useStraightLine = true           // <-- was isUseStraightLine = true
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


    private fun generateTrajectoryName(): String {
        val dateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault())
        return "trajectory_${dateFormat.format(Date())}"
    }

    // Toggle UI state
    private fun Boolean.updateUIState(trajectoryLoaded: Boolean) {
        binding?.apply {
            // Recording controls
            btnStartRecording.isEnabled = !this@updateUIState
            btnStopRecording.isEnabled = this@updateUIState
            btnCaptureWaypoint.isEnabled = this@updateUIState
            btnSaveTrajectory.isEnabled = !this@updateUIState && recordedWaypoints.isNotEmpty()

            // Replay controls
            btnLoadTrajectory.isEnabled = !this@updateUIState && curMissionPath.isNotEmpty()
            btnStartReplay.isEnabled = !this@updateUIState && trajectoryLoaded
            btnPauseReplay.isEnabled = !this@updateUIState && trajectoryLoaded
            btnStopReplay.isEnabled = !this@updateUIState && trajectoryLoaded

            // Status text
            tvRecordingStatus.text = if (this@updateUIState) getString(R.string.recording) else getString(R.string.stopped)
        }
    }

    private fun observeAircraftLocation() {
        val aircraftLocationKey = KeyTools.createKey(FlightControllerKey.KeyAircraftLocation)
        KeyManager.getInstance().listen(aircraftLocationKey, this) { _, newValue ->
            newValue?.let {
                binding?.tvAircraftLocation?.text =
                    getString(R.string.aircraft_location_format, it.latitude, it.longitude)
            }
        }
    }

    @SuppressLint("SetTextI18n")
    private fun addWaylineExecutingInfoListener() {
        WaypointMissionManager.getInstance().addWaylineExecutingInfoListener { waylineExecutingInfo ->
            binding?.tvMissionProgress?.text =
                "WaylineID: ${waylineExecutingInfo.waylineID}, Current: ${waylineExecutingInfo.currentWaypointIndex}"
        }
    }

    private fun addWaypointActionListener() {
        WaypointMissionManager.getInstance().addWaypointActionListener(object : WaypointActionListener {
            @Deprecated("Deprecated in Java")
            override fun onExecutionStart(actionId: Int) {
                binding?.tvCurrentWaypoint?.text = getString(R.string.current_waypoint_format, actionId)
            }
            @SuppressLint("SetTextI18n")
            override fun onExecutionStart(actionGroup: Int, actionId: Int) {
                binding?.tvCurrentWaypoint?.text = "Action Group: $actionGroup, Action ID: $actionId"
            }
            @Deprecated("Deprecated in Java")
            override fun onExecutionFinish(actionId: Int, error: IDJIError?) {
                error?.let { ToastUtils.showToast("Action error: ${it.description()}") }
            }
            override fun onExecutionFinish(actionGroup: Int, actionId: Int, error: IDJIError?) {
                error?.let { ToastUtils.showToast("Action error: ${it.description()}") }
            }
        })
    }

    private fun generateCircularTrajectory(centerLocation: LocationCoordinate2D, currentRelHeightMeters: Double) {
        val radiusInMeters = 30.0 // 30-meter radius
        val numberOfWaypoints = 12 // 12 waypoints
        val earthRadiusMeters = 6_371_000.0

        recordedWaypoints.clear()

        for (i in 0 until numberOfWaypoints) {
            // Clockwise angle
            val angle = -(i * 2.0 * Math.PI / numberOfWaypoints)

            // Offsets in meters
            val dx = radiusInMeters * sin(angle) // east
            val dy = radiusInMeters * cos(angle) // north

            // Convert meters to degrees
            val dLat = (dy / earthRadiusMeters) * (180.0 / Math.PI)
            val dLng = (dx / (earthRadiusMeters * cos(Math.toRadians(centerLocation.latitude)))) * (180.0 / Math.PI)

            val waypointLat = centerLocation.latitude + dLat
            val waypointLng = centerLocation.longitude + dLng

            val wpLocation = LocationCoordinate2D(waypointLat, waypointLng)
            val wp = createWaypointFromLocation(wpLocation, currentRelHeightMeters, i)
            recordedWaypoints.add(wp)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        KeyManager.getInstance().cancelListen(this)
        binding = null
    }

    // ================== Debug helpers ==================

    private fun nowIso(): String {
        val fmt = SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US)
        fmt.timeZone = TimeZone.getTimeZone("UTC")
        return fmt.format(Date())
    }

    private fun logI(msg: String) {
        Log.i(logTag, msg)
        debugSb.appendLine("INFO  ${nowIso()}  $msg")
    }

    private fun logE(msg: String) {
        Log.e(logTag, msg)
        debugSb.appendLine("ERROR ${nowIso()}  $msg")
    }

    /**
     * Write a .log file next to the KMZ path.
     */
    private fun saveDebugLog(kmzPath: String, content: String) {
        runCatching {
            val kmz = File(kmzPath)
            val logFile = File(kmz.parentFile, kmz.nameWithoutExtension + "_debug.log")
            FileOutputStream(logFile, false).use { it.write(content.toByteArray()) }
        }.onFailure { e ->
            Log.e(logTag, "Failed to save debug log: ${e.message}")
        }
    }

    private fun readEntry(zip: ZipFile, name: String): String? {
        val entry: ZipEntry = zip.getEntry(name) ?: return null
        return zip.getInputStream(entry).use { it.readBytes().toString(Charsets.UTF_8) }
    }

    /**
     * Quick check: list entries, lengths, and print top of files.
     */
    private fun dumpKmzQuickCheck(kmzPath: String, out: StringBuilder) {
        val f = File(kmzPath)
        if (!f.exists()) {
            out.appendLine("ERROR ${nowIso()}  KMZ not found: $kmzPath")
            return
        }
        out.appendLine("INFO  ${nowIso()}  KMZ at $kmzPath size=${f.length()} bytes")
        runCatching {
            ZipFile(f).use { zip ->
                val names = zip.entries().toList().map { it.name }
                out.appendLine("INFO  ${nowIso()}  KMZ entries: $names")
                val wpml = readEntry(zip, "wpmz/waylines.wpml")
                val tmpl = readEntry(zip, "wpmz/template.kml")
                out.appendLine("INFO  ${nowIso()}  waylines.wpml present=${wpml != null} len=${wpml?.length ?: 0}")
                out.appendLine("INFO  ${nowIso()}  template.kml present=${tmpl != null} len=${tmpl?.length ?: 0}")
                wpml?.let {
                    val head = it.lineSequence().take(8).joinToString("\n")
                    out.appendLine("INFO  ${nowIso()}  waylines.wpml head:\n$head")
                }
                tmpl?.let {
                    val head = it.lineSequence().take(8).joinToString("\n")
                    out.appendLine("INFO  ${nowIso()}  template.kml head:\n$head")
                }
            }
        }.onFailure { e ->
            out.appendLine("ERROR ${nowIso()}  dumpKmzQuickCheck failed: ${e.message}")
        }
    }

    /**
     * Pull <wpml:waylineId> values out of waylines.wpml by simple regex.
     */
    private fun extractWaylineIdsFromKmz(kmzPath: String): ArrayList<Int> {
        val result = arrayListOf<Int>()
        runCatching {
            ZipFile(File(kmzPath)).use { zip ->
                val wpml = readEntry(zip, "wpmz/waylines.wpml") ?: return@use
                // very tolerant regex for integers inside waylineId tag
                val regex = Regex("<\\s*wpml:waylineId\\s*>\\s*([0-9]+)\\s*<\\s*/\\s*wpml:waylineId\\s*>")
                regex.findAll(wpml).forEach { m ->
                    m.groupValues.getOrNull(1)?.toIntOrNull()?.let { result.add(it) }
                }
            }
        }
        return result
    }

    /**
     * Try reading wayline IDs through the SDK parser using reflection.
     */
    private fun getWaylineIdsViaSdk(kmzPath: String): ArrayList<Int> {
        val list = arrayListOf<Int>()
        runCatching {
            val kmzInfo = WPMZManager.getInstance().getKMZInfo(kmzPath)
            val parseInfo = kmzInfo.waylineWaylinesParseInfo
            val waylines = parseInfo?.waylines ?: return@runCatching
            logI("SDK parse: waylines=${waylines.size}")
            waylines.forEachIndexed { idx, wl ->
                var id: Int? = null
                // Try getter
                runCatching {
                    val m = wl.javaClass.methods.firstOrNull { it.name == "getWaylineID" && it.parameterTypes.isEmpty() }
                    id = (m?.invoke(wl) as? Int)
                }
                // Try fields
                if (id == null) {
                    id = runCatching { wl.javaClass.getDeclaredField("waylineID").apply { isAccessible = true }.get(wl) as? Int }.getOrNull()
                        ?: runCatching { wl.javaClass.getDeclaredField("id").apply { isAccessible = true }.get(wl) as? Int }.getOrNull()
                }
                val wpCount = runCatching {
                    val m = wl.javaClass.methods.firstOrNull { it.name == "getWaypoints" && it.parameterTypes.isEmpty() }
                    val listAny = m?.invoke(wl) as? List<*>
                    listAny?.size ?: -1
                }.getOrNull() ?: -1
                logI("SDK parse: wayline[$idx] id=$id waypoints=$wpCount")
                if (id != null && id!! >= 0) list.add(id!!)
            }
        }
        return list
    }

    private fun appendLogWaylineIds(kmzPath: String, out: StringBuilder) {
        val ids = extractWaylineIdsFromKmz(kmzPath)
        out.appendLine("INFO  ${nowIso()}  Wayline IDs from waylines.wpml: $ids")
    }

    private fun appendLogSdkParse(kmzPath: String, out: StringBuilder) {
        val ids = getWaylineIdsViaSdk(kmzPath)
        if (ids.isEmpty()) return
        out.appendLine("INFO  ${nowIso()}  Extracted wayline IDs: $ids")
    }
}
