package dji.sampleV5.aircraft.logger

import android.os.Build
import android.os.Environment
import android.util.Log
import dji.v5.utils.common.ContextUtil
import org.json.JSONObject
import java.io.File
import java.io.FileWriter
import java.io.PrintWriter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

/**
 * WildBridgeFlightLogger — persists flight event data to a location that survives
 * app uninstalls and device reflashing.
 *
 * === WildBridge JSONL logs ===
 * Written to the best available storage (priority order):
 *   1. Removable microSD card root → WildBridge/FlightLogs/YYYY-MM-DD/ (needs MANAGE_EXTERNAL_STORAGE on API 30+)
 *   2. Documents/WildBridge/FlightLogs/YYYY-MM-DD/  (needs MANAGE_EXTERNAL_STORAGE on API 30+)
 *   3. Android/data/<pkg>/files/FlightLogs/YYYY-MM-DD/  (app-external fallback — deleted on uninstall)
 *
 * One JSONL file is created per flight session. Each line is one JSON event:
 *   {"t":1711188600,"type":"SESSION_START","drone":"scout"}
 *   {"t":1711188610,"type":"COMMAND","cmd":"/send/goto","params":"48.85,2.35,50"}
 *   {"t":1711188615,"type":"TELEMETRY","speed":5.1,"heading":90.0,"batteryLevel":82,...}
 *   {"t":1711188700,"type":"STATUS","status":"RETURNING_HOME"}
 *   {"t":1711188750,"type":"SESSION_END","reason":"landed"}
 *
 * === DJI TXT flight records ===
 * The SDK writes these automatically to:
 *   Android/data/com.dji.sampleV5.aircraft/files/DJI/FlightRecord/
 * That folder lives in app-specific external storage and is deleted on uninstall.
 * On every app launch (and after landing) we mirror any new files to:
 *   WildBridge/DJI_FlightRecords/  (same storage-priority chain as above)
 * Already-copied files are skipped by filename, so the sync is always safe to repeat.
 */
object WildBridgeFlightLogger {

    private const val TAG = "WildBridgeFlightLogger"

    @Volatile
    var droneName: String = "unknown"
        private set

    @Volatile private var writer: PrintWriter? = null
    @Volatile private var logFile: File? = null
    @Volatile private var sessionActive = false

    /** True while a log file is open. */
    val isSessionActive: Boolean get() = sessionActive

    /** Absolute path of the current log file, or null if no session is open. */
    val currentLogPath: String? get() = logFile?.absolutePath

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Set (or update) the drone name used in log file names.
     * Safe characters only — everything else is replaced with '_'.
     */
    fun setDroneName(name: String) {
        droneName = name.trim().replace(Regex("[^a-zA-Z0-9._-]"), "_").ifBlank { "unknown" }
    }

    /**
     * Open a new JSONL log file and write the SESSION_START marker.
     * If a session is already open it is closed first (handles missed landing events).
     */
    fun startSession() {
        if (sessionActive) endSession("session_restart")
        try {
            val dir = resolveLogDir()
            if (dir == null) {
                Log.e(TAG, "Cannot resolve log directory — logging disabled for this session")
                return
            }
            val timeStr = SimpleDateFormat("HH-mm-ss", Locale.US).format(Date())
            val file = File(dir, "${timeStr}_${droneName}.jsonl")
            logFile = file
            writer = PrintWriter(FileWriter(file, /* append = */ false))
            sessionActive = true
            commitLog("SESSION_START", mapOf("drone" to droneName, "logDir" to dir.absolutePath))
            Log.i(TAG, "Flight log started → ${file.absolutePath}")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start flight log: ${e.message}")
        }
    }

    /** Write SESSION_END and close the file. */
    fun endSession(reason: String = "app_stopped") {
        if (!sessionActive) return
        commitLog("SESSION_END", mapOf("reason" to reason))
        flushAndClose()
        sessionActive = false
        Log.i(TAG, "Flight log ended ($reason) → ${logFile?.absolutePath}")
    }

    /**
     * Log a telemetry snapshot.
     * Accepts the raw JSON string from VirtualStickFragment.getTelemetryJson() and
     * splices in "t" and "type" fields without a full re-parse.
     * Call at most every 5 seconds to keep file sizes manageable.
     */
    fun logTelemetry(telemetryJson: String) {
        if (!sessionActive || telemetryJson.isBlank()) return
        val ts = System.currentTimeMillis() / 1000
        // Splice timestamp + type into the existing flat JSON object
        val inner = telemetryJson.trim().removePrefix("{").removeSuffix("}")
        writeLine("""{"t":$ts,"type":"TELEMETRY",$inner}""")
    }

    /**
     * Log an HTTP command.
     * [endpoint] is the URI (e.g. "/send/goto"), [params] is the POST body.
     */
    fun logCommand(endpoint: String, params: String = "") {
        val fields = mutableMapOf<String, Any>("cmd" to endpoint)
        if (params.isNotBlank()) fields["params"] = params.take(300) // cap length
        commitLog("COMMAND", fields)
    }

    /** Log a drone status / mode change (e.g. "NAVIGATING", "RETURNING_HOME"). */
    fun logStatus(status: String) {
        commitLog("STATUS", mapOf("status" to status))
    }

    /**
     * Sync DJI SDK-managed flight records from [djiLogPath] to the WildBridge log root.
     *
     * This is a simple one-way mirror: any TXT/CSV/CLOG file in the DJI folder that does
     * not already exist (by name) in the WildBridge destination is copied over.
     * Existing files are left untouched (no overwrite), so the call is safe to run at
     * every app launch without duplicating anything.
     *
     * Returns the number of files newly copied.
     */
    fun syncDjiFlightLogs(djiLogPath: String): Int {
        if (djiLogPath.isBlank()) {
            Log.w(TAG, "syncDjiFlightLogs: empty source path")
            return 0
        }
        val sourceDir = File(djiLogPath)
        if (!sourceDir.isDirectory) {
            Log.w(TAG, "syncDjiFlightLogs: source does not exist: $djiLogPath")
            return 0
        }
        val destDir = resolveDjiSyncDir()
        if (destDir == null) {
            Log.e(TAG, "syncDjiFlightLogs: could not create destination directory")
            return 0
        }
        return try {
            // Walk the tree recursively — DJI SDK may place records in subdirectories.
            val allFiles = sourceDir.walk().filter { f ->
                f.isFile && f.extension.lowercase() in setOf("txt", "csv", "clog")
            }

            var copied = 0
            for (src in allFiles) {
                val dest = File(destDir, src.name)
                if (!dest.exists()) {
                    try {
                        src.copyTo(dest)
                        Log.i(TAG, "DJI log synced: ${src.name}")
                        copied++
                    } catch (e: Exception) {
                        Log.w(TAG, "Failed to sync ${src.name}: ${e.message}")
                    }
                }
            }
            Log.i(TAG, "syncDjiFlightLogs: $copied new file(s) copied to ${destDir.absolutePath}")
            copied
        } catch (e: Exception) {
            Log.e(TAG, "syncDjiFlightLogs error: ${e.message}")
            0
        }
    }

    /**
     * Resolve the fixed directory where DJI TXT logs are synced.
     * Uses the same storage-priority chain as [resolveLogDir] but writes to
     * `WildBridge/DJI_FlightRecords/` — a flat, date-independent folder so all
     * DJI logs live in one place regardless of when they were produced.
     */
    private fun resolveDjiSyncDir(): File? {
        val subPath = "WildBridge/DJI_FlightRecords"
        val ctx = ContextUtil.getContext()
        val hasFullAccess = Build.VERSION.SDK_INT < Build.VERSION_CODES.R ||
                Environment.isExternalStorageManager()

        // 1. Removable SD card root — survives both uninstalls and reflashing.
        //    Requires MANAGE_EXTERNAL_STORAGE on Android 11+ to write outside app-private.
        if (hasFullAccess) {
            try {
                val externalDirs = ctx.getExternalFilesDirs(null)
                for (i in 1 until externalDirs.size) {
                    val d = externalDirs[i] ?: continue
                    var root = d; repeat(4) { root = root.parentFile ?: root }
                    val dir = File(root, subPath)
                    if (dir.mkdirs() || dir.isDirectory) {
                        Log.i(TAG, "resolveDjiSyncDir: using SD card root: ${dir.absolutePath}")
                        return dir
                    }
                }
            } catch (e: Exception) {
                Log.w(TAG, "resolveDjiSyncDir: SD card root check failed: ${e.message}")
            }
        }

        // 2. Documents folder — survives uninstalls; needs MANAGE_EXTERNAL_STORAGE on API 30+.
        if (hasFullAccess) {
            val dir = File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), subPath)
            if (dir.mkdirs() || dir.isDirectory) {
                Log.i(TAG, "resolveDjiSyncDir: using Documents: ${dir.absolutePath}")
                return dir
            }
        }

        // 3. App-external fallback — does NOT survive uninstalls.
        Log.w(TAG, "resolveDjiSyncDir: MANAGE_EXTERNAL_STORAGE not granted, falling back to app-external")
        return try {
            val dir = File(ctx.getExternalFilesDir(null), "DJI_FlightRecords")
            if (dir.mkdirs() || dir.isDirectory) {
                Log.w(TAG, "resolveDjiSyncDir: using app-external fallback: ${dir.absolutePath}")
                dir
            } else {
                Log.e(TAG, "resolveDjiSyncDir: all storage options failed")
                null
            }
        } catch (e: Exception) {
            Log.e(TAG, "resolveDjiSyncDir: fallback failed: ${e.message}")
            null
        }
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Resolve the directory where the log file for today's date should live.
     *
     * Priority chain (highest to lowest durability):
     *   1. Removable microSD card root — survives both uninstalls and OS reflashing.
     *      Requires MANAGE_EXTERNAL_STORAGE on Android 11+ (requested at startup).
     *   2. Public Documents folder — survives app uninstalls; visible in file managers.
     *      Requires MANAGE_EXTERNAL_STORAGE on Android 11+ (requested at startup).
     *   3. App-external files dir — does NOT survive uninstalls (last-resort fallback).
     */
    private fun resolveLogDir(): File? {
        val dateStr = SimpleDateFormat("yyyy-MM-dd", Locale.US).format(Date())
        val subPath = "WildBridge/FlightLogs/$dateStr"
        val ctx = ContextUtil.getContext()
        val hasFullAccess = Build.VERSION.SDK_INT < Build.VERSION_CODES.R ||
                Environment.isExternalStorageManager()

        // 1. Removable SD card root — survives both uninstalls and reflashing.
        //    Requires MANAGE_EXTERNAL_STORAGE on Android 11+ to write outside app-private.
        if (hasFullAccess) {
            try {
                val externalDirs = ctx.getExternalFilesDirs(null)
                for (i in 1 until externalDirs.size) {
                    val appPrivateOnCard = externalDirs[i] ?: continue
                    var cardRoot: File = appPrivateOnCard
                    repeat(4) { cardRoot = cardRoot.parentFile ?: cardRoot }
                    val dir = File(cardRoot, subPath)
                    if (dir.mkdirs() || dir.isDirectory) {
                        Log.i(TAG, "Using removable SD card root: ${dir.absolutePath}")
                        return dir
                    }
                }
            } catch (e: Exception) {
                Log.w(TAG, "Removable SD card root check failed: ${e.message}")
            }
        }

        // 2. Public Documents/WildBridge/FlightLogs — needs MANAGE_EXTERNAL_STORAGE on API 30+.
        if (hasFullAccess) {
            val documentsRoot = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS)
            val dir = File(documentsRoot, "WildBridge/FlightLogs/$dateStr")
            if (dir.mkdirs() || dir.isDirectory) {
                Log.i(TAG, "Using Documents folder: ${dir.absolutePath}")
                return dir
            }
        }

        // 3. App-external fallback — does NOT survive uninstalls.
        Log.w(TAG, "MANAGE_EXTERNAL_STORAGE not granted, falling back to app-external files dir")
        return try {
            val fallback = File(ctx.getExternalFilesDir(null), "FlightLogs/$dateStr")
            fallback.mkdirs()
            if (fallback.isDirectory) fallback else null
        } catch (e: Exception) {
            Log.e(TAG, "Cannot resolve any log directory: ${e.message}")
            null
        }
    }

    private fun commitLog(type: String, fields: Map<String, Any> = emptyMap()) {
        if (!sessionActive) return
        try {
            val obj = JSONObject()
            obj.put("t", System.currentTimeMillis() / 1000)
            obj.put("type", type)
            fields.forEach { (k, v) -> obj.put(k, v) }
            writeLine(obj.toString())
        } catch (e: Exception) {
            Log.w(TAG, "commitLog error: ${e.message}")
        }
    }

    @Synchronized
    private fun writeLine(line: String) {
        try {
            writer?.println(line)
            writer?.flush()
        } catch (e: Exception) {
            Log.w(TAG, "writeLine error: ${e.message}")
        }
    }

    @Synchronized
    private fun flushAndClose() {
        try {
            writer?.flush()
            writer?.close()
        } catch (e: Exception) {
            Log.w(TAG, "flushAndClose error: ${e.message}")
        } finally {
            writer = null
        }
    }
}
