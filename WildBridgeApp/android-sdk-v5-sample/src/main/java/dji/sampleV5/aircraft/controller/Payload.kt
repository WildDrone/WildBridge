package dji.sampleV5.aircraft.controller

import android.util.Log
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.value.camera.LaserMeasureInformation
import dji.sdk.keyvalue.value.camera.LaserMeasureState
import dji.sdk.keyvalue.value.camera.LaserWorkMode
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen
import dji.v5.et.set
import dji.v5.manager.KeyManager

/**
 * H20T payload control.
 *
 * Singleton so the HTTP endpoints in WildBridgeDefaultLayoutActivity can fire payload
 * actions without holding payload-specific state in the activity. Blocking calls
 * (e.g. takeFreshLrfReading) are invoked from the HTTP server's worker threads.
 *
 * Currently houses the Laser Range Finder (LRF); thermal capture will move here later.
 */
object Payload {

    private const val TAG = "Payload"

    // ==================== Laser Range Finder (LRF) ====================

    private val laserKey: DJIKey<LaserWorkMode> = CameraKey.KeyLaserWorkMode.create()
    private val laserMeasureKey: DJIKey<LaserMeasureInformation> = CameraKey.KeyLaserMeasureInformation.create()
    private val lrfReadingLock = Any()

    @Volatile
    private var lrfInfo: LaserMeasureInformation? = null
    @Volatile
    private var lrfListenerRegistered: Boolean = false

    // Register a persistent listener that caches the latest laser measurement. Idempotent.
    private fun setupLaserMeasureListener() {
        if (lrfListenerRegistered) return
        // Cache the latest measurement as a fallback; takeFreshLrfReading polls the key directly.
        laserMeasureKey.listen(this) { newValue: LaserMeasureInformation? ->
            lrfInfo = newValue
        }
        lrfListenerRegistered = true
        Log.i(TAG, "LRF measure listener registered")
    }

    // Fire the laser, wait for a fresh measurement, then turn the laser off.
    // Blocking, call from a worker thread.
    fun takeFreshLrfReading(timeoutMs: Long = 2000L): LaserMeasureInformation? = synchronized(lrfReadingLock) {
        setupLaserMeasureListener()
        lrfInfo = null

        try {
            laserKey.set(
                LaserWorkMode.OPEN_ALWAYS,
                onSuccess = { Log.i(TAG, "LRF laser opened") },
                onFailure = { error -> Log.e(TAG, "LRF laser open failed: ${error.description()}") }
            )

            // Poll the key directly for fresh values; wait for the laser to lock (state == NORMAL)
            val deadline = System.currentTimeMillis() + timeoutMs
            var reading: LaserMeasureInformation? = null
            while (System.currentTimeMillis() < deadline) {
                val current = laserMeasureKey.get() ?: lrfInfo
                if (current != null) {
                    reading = current
                    if (current.laserMeasureState == LaserMeasureState.NORMAL) break
                }
                try {
                    Thread.sleep(50)
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                    break
                }
            }
            if (reading == null) {
                Log.w(TAG, "LRF reading timed out after ${timeoutMs}ms (no data from laser)")
            } else {
                Log.i(TAG, "LRF reading: ${reading.distance} m, state=${reading.laserMeasureState}")
            }
            reading
        } finally {
            // Return the laser to on-demand (closed)
            laserKey.set(
                LaserWorkMode.OPEN_ON_DEMAND,
                onSuccess = { Log.i(TAG, "LRF laser set to OPEN_ON_DEMAND (off)") },
                onFailure = { error -> Log.e(TAG, "LRF laser close failed: ${error.description()}") }
            )
        }
    }

    // Cancel payload key listeners. Call from the host activity's onDestroy.
    fun destroy() {
        KeyManager.getInstance().cancelListen(this)
        lrfListenerRegistered = false
        lrfInfo = null
    }
}
