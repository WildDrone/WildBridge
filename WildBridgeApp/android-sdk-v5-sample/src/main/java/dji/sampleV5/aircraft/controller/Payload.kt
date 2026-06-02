package dji.sampleV5.aircraft.controller

import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sampleV5.aircraft.models.MediaVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.GeneratedMediaFileInfo
import dji.sdk.keyvalue.value.camera.LaserMeasureInformation
import dji.sdk.keyvalue.value.camera.LaserMeasureState
import dji.sdk.keyvalue.value.camera.LaserWorkMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen
import dji.v5.et.set
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.media.MediaFile
import dji.v5.manager.datacenter.media.MediaFileDownloadListener
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DiskUtil
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStream
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * H20T payload control — Laser Range Finder (LRF) and thermal photo capture.
 *
 * Singleton so the HTTP endpoints in WildBridgeDefaultLayoutActivity can fire payload
 * actions without holding payload-specific state in the activity. The blocking calls
 * (takeFreshLrfReading, takeThermalImage, sendMediaFile) are invoked from the HTTP
 * server's worker threads, and are expected to run one at a time — the command server
 * issues capture requests serially.
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

    // ==================== Thermal photo capture (H20T) ====================

    private val mainHandler = Handler(Looper.getMainLooper())

    // Fires when the camera writes a new photo to the SD card; carries the new file's index.
    private val keyNewlyGeneratedMediaFile = KeyTools.createKey(CameraKey.KeyNewlyGeneratedMediaFile)
    @Volatile
    private var latestGeneratedMediaInfo: GeneratedMediaFileInfo? = null
    @Volatile
    private var newMediaListenerRegistered = false

    // Persistent listener caching the most-recent generated-file info. Registered once
    // (idempotent) and torn down via destroy(). A fresh listener per capture would replay the
    // PREVIOUS photo's cached value on registration, which is what made repeated captures return
    // the same stale file.
    private fun setupNewMediaListener() {
        if (newMediaListenerRegistered) return
        keyNewlyGeneratedMediaFile.listen(this) { newValue: GeneratedMediaFileInfo? ->
            latestGeneratedMediaInfo = newValue
        }
        newMediaListenerRegistered = true
    }

    // Capture a thermal photo on the H20T and return the resulting on-camera MediaFile.
    // Blocking, call from a worker thread. mediaVM must be init'd with SD card storage and the
    // LEFT_OR_MAIN component index (done in the host activity's onCreate).
    fun takeThermalImage(mediaVM: MediaVM): MediaFile? {
        try {
            setupNewMediaListener()

            // Remember the current newest-photo index (from the key cache or the last
            // listener push) so the shot we are about to take can be told apart from it.
            val baselineIndex = (keyNewlyGeneratedMediaFile.get() ?: latestGeneratedMediaInfo)?.index
            latestGeneratedMediaInfo = null

            // Trip the shutter on the main thread and wait for the SDK callback.
            var photoError: String? = null
            val photoLatch = CountDownLatch(1)
            mainHandler.post {
                mediaVM.takePhoto(object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        photoLatch.countDown()
                    }
                    override fun onFailure(error: IDJIError) {
                        photoError = error.description()
                        Log.e(TAG, "Photo capture failed: $photoError")
                        photoLatch.countDown()
                    }
                })
            }
            if (!photoLatch.await(8, TimeUnit.SECONDS)) {
                Log.e(TAG, "Timeout waiting for shutter")
                return null
            }
            if (photoError != null) return null

            // Wait for a new index to appear from either the listener push or the key cache
            var newIndex: Int? = null
            val idxDeadline = System.currentTimeMillis() + 8000L
            while (System.currentTimeMillis() < idxDeadline) {
                // Accept a fresh index from EITHER the listener push or the key cache,
                // whichever first reports something different from the baseline
                val cur = listOfNotNull(
                    latestGeneratedMediaInfo?.index,
                    keyNewlyGeneratedMediaFile.get()?.index
                ).firstOrNull { it != baselineIndex }
                if (cur != null) {
                    newIndex = cur
                    break
                }
                Thread.sleep(100)
            }
            if (newIndex == null) {
                Log.e(TAG, "No new media index after shutter (baseline=$baselineIndex)")
                return null
            }
            Log.i(TAG, "New photo index=$newIndex (baseline=$baselineIndex)")

            // Pull that single entry and poll the file list until it appears.
            mainHandler.post { mediaVM.pullMediaFileListFromCamera(newIndex, 1) }
            val listDeadline = System.currentTimeMillis() + 10000L
            while (System.currentTimeMillis() < listDeadline) {
                val file = mediaVM.mediaFileListData.value?.data?.firstOrNull { it.fileIndex == newIndex }
                if (file != null) {
                    Log.i(TAG, "Resolved ${file.fileName} at index ${file.fileIndex}")
                    return file
                }
                Thread.sleep(100)
            }
            Log.e(TAG, "File list never surfaced index $newIndex")
            return null
        } catch (e: Exception) {
            Log.e(TAG, "Error taking thermal image: ${e.message}", e)
            return null
        } finally {
            // takePhoto switches the camera to PHOTO_NORMAL; restore video mode so the
            // live feed isn't left in photo mode after a capture.
            mainHandler.post {
                mediaVM.setVideoMode(object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        Log.i(TAG, "Camera restored to video mode")
                    }
                    override fun onFailure(error: IDJIError) {
                        Log.e(TAG, "Failed to restore video mode: ${error.description()}")
                    }
                })
            }
        }
    }

    // Download photoFile from the camera to local cache, then stream the raw JPEG bytes back
    // over outputStream as a jpeg HTTP response. Blocking.
    fun sendMediaFile(photoFile: MediaFile, outputStream: OutputStream) {
        try {
            Log.i(TAG, "Downloading media file: ${photoFile.fileName}")

            val dirs = File(DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), "/mediafile"))
            if (!dirs.exists()) dirs.mkdirs()

            val filepath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), "/mediafile/" + photoFile.fileName)
            val file = File(filepath)
            if (file.exists()) file.delete()

            val downloadLatch = CountDownLatch(1)
            var downloadSuccess = false
            var downloadError: String? = null

            val outputFileStream = FileOutputStream(file, false)
            val bos = BufferedOutputStream(outputFileStream)

            photoFile.pullOriginalMediaFileFromCamera(0L, object : MediaFileDownloadListener {
                override fun onStart() {
                    Log.i(TAG, "Download started: ${photoFile.fileName}")
                }
                override fun onProgress(total: Long, current: Long) { /* no-op */ }
                override fun onRealtimeDataUpdate(data: ByteArray, position: Long) {
                    try {
                        bos.write(data)
                        bos.flush()
                    } catch (e: IOException) {
                        Log.e(TAG, "Error writing download data: ${e.message}")
                        downloadError = e.message
                    }
                }
                override fun onFinish() {
                    try {
                        bos.close()
                        outputFileStream.close()
                        downloadSuccess = true
                    } catch (error: IOException) {
                        downloadError = error.message
                    } finally {
                        downloadLatch.countDown()
                    }
                }
                override fun onFailure(error: IDJIError?) {
                    try {
                        bos.close()
                        outputFileStream.close()
                    } catch (e: IOException) { /* ignore close errors */ }
                    downloadError = error?.description() ?: "Unknown download error"
                    Log.e(TAG, "Download failed: $downloadError")
                    downloadLatch.countDown()
                }
            })

            if (!downloadLatch.await(60, TimeUnit.SECONDS)) {
                Log.e(TAG, "Timeout waiting for file download")
                sendErrorResponse(outputStream, "Download timeout")
                return
            }
            if (!downloadSuccess || downloadError != null) {
                sendErrorResponse(outputStream, "Download failed: $downloadError")
                return
            }

            val actualFileSize = file.length()
            if (!file.exists() || actualFileSize == 0L) {
                sendErrorResponse(outputStream, "Downloaded file missing or empty")
                return
            }
            Log.i(TAG, "Downloaded ${photoFile.fileName} ($actualFileSize bytes)")

            val headers = StringBuilder().apply {
                append("HTTP/1.1 200 OK\r\n")
                append("Content-Type: image/jpeg\r\n")
                append("Content-Length: $actualFileSize\r\n")
                append("Access-Control-Allow-Origin: *\r\n")
                append("Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n")
                append("Access-Control-Allow-Headers: Content-Type\r\n")
                append("\r\n")
            }
            outputStream.write(headers.toString().toByteArray())

            file.inputStream().use { input ->
                val buffer = ByteArray(8192)
                var bytesRead: Int
                while (input.read(buffer).also { bytesRead = it } != -1) {
                    outputStream.write(buffer, 0, bytesRead)
                }
                outputStream.flush()
            }
            Log.i(TAG, "Thermal image sent (${photoFile.fileName})")
            mainHandler.post { ToastUtils.showToast("Thermal image sent") }
        } catch (e: Exception) {
            Log.e(TAG, "Error sending file: ${e.message}", e)
            try {
                sendErrorResponse(outputStream, "Error sending file: ${e.message}")
            } catch (responseError: Exception) {
                Log.e(TAG, "Failed to send error response: ${responseError.message}")
            }
        }
    }

    fun sendErrorResponse(outputStream: OutputStream, message: String) {
        val body = message.toByteArray()
        val headers = StringBuilder().apply {
            append("HTTP/1.1 500 Internal Server Error\r\n")
            append("Content-Type: text/plain\r\n")
            append("Content-Length: ${body.size}\r\n")
            append("Access-Control-Allow-Origin: *\r\n")
            append("\r\n")
        }
        outputStream.write(headers.toString().toByteArray())
        outputStream.write(body)
        outputStream.flush()
    }

    // Cancel payload key listeners. Call from the host activity's onDestroy.
    fun destroy() {
        KeyManager.getInstance().cancelListen(this)
        lrfListenerRegistered = false
        lrfInfo = null
        newMediaListenerRegistered = false
        latestGeneratedMediaInfo = null
    }
}
