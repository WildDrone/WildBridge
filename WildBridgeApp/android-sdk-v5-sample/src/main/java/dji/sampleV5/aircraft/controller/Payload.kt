package dji.sampleV5.aircraft.controller

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sampleV5.aircraft.models.MediaVM
import dji.sampleV5.aircraft.models.PayloadWidgetVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.CameraType
import dji.sdk.keyvalue.value.camera.DCFCameraType
import dji.sdk.keyvalue.value.camera.GeneratedMediaFileInfo
import dji.sdk.keyvalue.value.camera.LaserMeasureInformation
import dji.sdk.keyvalue.value.camera.LaserMeasureState
import dji.sdk.keyvalue.value.camera.LaserWorkMode
import dji.sdk.keyvalue.value.camera.MediaFileType
import dji.sdk.keyvalue.value.file.FileListRequestTimeOrderType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen
import dji.v5.et.set
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.payload.PayloadIndexType
import dji.v5.manager.datacenter.media.MediaFile
import dji.v5.manager.datacenter.media.MediaFileDownloadListener
import dji.sdk.keyvalue.value.payload.WidgetType
import dji.sdk.keyvalue.value.payload.WidgetValue
import java.io.OutputStream
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import kotlin.math.roundToInt

/**
 * Hybrid-payload control — Laser Range Finder (LRF) and multi-lens photo capture.
 *
 * Supports the Zenmuse H20T, H20N and H30T. All three are multi-lens hybrid payloads that
 * write one JPEG per lens per shutter, tagged with a filename suffix (_T thermal, _W/_V wide,
 * _Z zoom); the capture path classifies by that shared suffix scheme and ignores any extra
 * lenses (H30T near-infrared, H20N second thermal) and non-image sidecars that don't map to
 * the wide/zoom/thermal slots the bridge exposes.
 *
 */
object Payload {

    private const val TAG = "Payload"

    // ==================== Laser Range Finder (LRF) ====================

    private val laserKey: DJIKey<LaserWorkMode> = CameraKey.KeyLaserWorkMode.create()

    // Master on/off for the rangefinder — the same toggle DJI Pilot 2 exposes. KeyLaserWorkMode
    // only selects the firing mode (on-demand vs. always); measurement stays OFF (every
    // LaserMeasureInformation field null) until this is set true. Enabling it here removes the need
    // to flip the laser on manually in DJI Pilot 2.
    private val laserEnabledKey: DJIKey<Boolean> = CameraKey.KeyLaserMeasureEnabled.create()

    // Enable the rangefinder and set it to fire continuously. Idempotent; safe to call repeatedly.
    private fun enableLaser() {
        laserEnabledKey.set(
            true,
            onSuccess = { Log.i(TAG, "LRF measure enabled") },
            onFailure = { error -> Log.e(TAG, "LRF measure enable failed: ${error.description()}") }
        )
        laserKey.set(
            LaserWorkMode.OPEN_ALWAYS,
            onSuccess = { Log.i(TAG, "LRF laser opened") },
            onFailure = { error -> Log.e(TAG, "LRF laser open failed: ${error.description()}") }
        )
    }

    init { enableLaser() }

    // ==================== Thermal sensor "sun protection" ====================

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
            // Re-apply: the init-time enable runs before the SDK is connected and silently fails,
            // so the first real reading must turn the laser on itself.
            enableLaser()

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
//            // Return the laser to on-demand (closed)
//            laserKey.set(
//                LaserWorkMode.OPEN_ON_DEMAND,
//                onSuccess = { Log.i(TAG, "LRF laser set to OPEN_ON_DEMAND (off)") },
//                onFailure = { error -> Log.e(TAG, "LRF laser close failed: ${error.description()}") } )
        }
    }
    // ==================== Multi-lens photo capture (H20T / H20N / H30T) ====================

    private val mainHandler = Handler(Looper.getMainLooper())
    private const val MEDIA_PULL_TIMEOUT_MS = 6000L      // per single list refresh
    private const val CAPTURE_OVERALL_TIMEOUT_MS = 30000L // whole capture resolution

    private const val CAPTURE_PULL_COUNT = 16
    private const val NARROW_PULL_FALLBACK_TRIES = 4

    // Event-driven resolve (KeyNewlyGeneratedMediaFile). The camera pushes one event per file the
    // shutter writes — index + lens (dcf_type) but NO downloadable handle. We wait on these cheap
    // local signals (zero camera round trips) to learn the shot's files have landed, then do ONE
    // list pull to materialise the handles.
    private const val EVENT_SETTLE_MS = 1500L       // no new file event this long => shot's files all landed
    private const val EVENT_FIRST_TIMEOUT_MS = 6000L // no event AT ALL this long => key lagged; use pull loop

    @Volatile
    private var narrowPullSupported = true

    // Identifies the attached payload so capture can adapt to it. LEFT_OR_MAIN component index,
    // matching the mediaVM the host activity wires up. Null until the camera is connected.
    private val cameraTypeKey: DJIKey<CameraType> = CameraKey.KeyCameraType.create()
    private fun activeCameraType(): CameraType? = cameraTypeKey.get()

    // Fires when the camera writes a new photo to the SD card; carries the new file's index.
    private val keyNewlyGeneratedMediaFile = KeyTools.createKey(CameraKey.KeyNewlyGeneratedMediaFile)
    @Volatile
    private var latestGeneratedMediaInfo: GeneratedMediaFileInfo? = null
    @Volatile
    private var newMediaListenerRegistered = false

    // During a capture, every KeyNewlyGeneratedMediaFile push is queued here so captureNewMediaFiles
    // can block on cheap local signals instead of re-pulling the media list once per file.
    private val mediaEventQueue = java.util.concurrent.LinkedBlockingQueue<GeneratedMediaFileInfo>()
    @Volatile
    private var collectingEvents = false

    private fun setupNewMediaListener() {
        if (newMediaListenerRegistered) return
        keyNewlyGeneratedMediaFile.listen(this) { newValue: GeneratedMediaFileInfo? ->
            latestGeneratedMediaInfo = newValue
            if (collectingEvents && newValue != null) mediaEventQueue.offer(newValue)
        }
        newMediaListenerRegistered = true
    }

    // Lens of a pushed event, from its DCF camera type (handle-free; no list pull needed).
    private fun isThermalLens(t: DCFCameraType?): Boolean = t == DCFCameraType.INFRARED
    private fun isWideLens(t: DCFCameraType?): Boolean =
        t == DCFCameraType.WIDE || t == DCFCameraType.VISIBLE || t == DCFCameraType.RGB
    private fun isZoomLens(t: DCFCameraType?): Boolean = t == DCFCameraType.ZOOM

    @Volatile
    private var mediaWarmedUp = false

    // Build the SD-card media list once, in the background, so the FIRST capture isn't cold.
    fun warmUpMedia(mediaVM: MediaVM) {
        if (mediaWarmedUp) return
        mediaWarmedUp = true
        Thread {
            try {
                setupNewMediaListener()
                // Generous cap: the cold first fetch can take far longer than a warm refresh.
                val files = mediaVM.pullAndAwait(CAPTURE_OVERALL_TIMEOUT_MS)
                Log.i(TAG, "Media warm-up complete: ${files.size} file(s) on card")
            } catch (e: Exception) {
                Log.w(TAG, "Media warm-up failed (will retry): ${e.message}")
                mediaWarmedUp = false
            }
        }.start()
    }

    // Allow the next connection to warm up again. Call when the aircraft disconnects.
    fun resetMediaWarmup() { mediaWarmedUp = false }

    // A single shutter on the H20T/H20N/H30T writes co-aligned files to the SD card with the same
    // timestamp and scene: a radiometric thermal R-JPEG, the wide visual photo, and the zoom-lens
    // photo.
    data class ThermalCapture(val thermal: MediaFile?, val visual: MediaFile?, val zoom: MediaFile?)

    // Per-lens filename suffixes (before the extension), shared by the H20T/H20N/H30T:
    // _T = thermal R-JPEG, _W = wide / _V = visible (RGB or starlight), _Z = zoom.
    private fun baseNameNoExt(name: String?): String =
        (name ?: "").substringBeforeLast('.').uppercase()

    private fun isThermalName(name: String?): Boolean = baseNameNoExt(name).endsWith("_T")

    private fun isWideName(name: String?): Boolean =
        baseNameNoExt(name).let { it.endsWith("_W") || it.endsWith("_V") }

    private fun isZoomName(name: String?): Boolean = baseNameNoExt(name).endsWith("_Z")

    // The co-exposed lens files of one shutter share a DCF base name and differ only by the lens
    // suffix (e.g. DJI_20260613223116_0001_T / _W / _Z). Stripping that suffix groups the siblings

    private val lensSuffixRegex = Regex("_[TWVZ]$")
    private fun lensGroupBase(name: String?): String =
        lensSuffixRegex.replace(baseNameNoExt(name), "")

    // Fire one shutter and return the thermal, wide-visual (RGB) and zoom MediaFiles from it.
    // Internal helper for captureThermal. Blocking, call from a worker thread.
    private fun takeThermalAndVisual(mediaVM: MediaVM): ThermalCapture {
        val newFiles = captureNewMediaFiles(mediaVM)
        if (newFiles.isEmpty()) return ThermalCapture(null, null, null)

        val variants = LinkedHashMap<String, MediaFile>()
        fun add(file: MediaFile?) {
            if (file?.fileName == null) return
            variants.putIfAbsent(file.fileName, file)
            file.subMediaFile?.forEach { sub -> sub?.fileName?.let { variants.putIfAbsent(it, sub) } }
        }
        newFiles.forEach { add(it) }

        val imageTypes = setOf(MediaFileType.JPEG, MediaFileType.DNG, MediaFileType.TIFF)
        val all = variants.values.filter { it.fileType in imageTypes }
            .ifEmpty { variants.values.toList() }
        Log.i(TAG, "Lens files this shutter (cam=${activeCameraType()}): " +
            all.joinToString { "${it.fileName}#${it.fileIndex}(${it.fileSize}B)" })

        // Thermal: the _T file by name; failing that the SMALLEST image (the radiometric R-JPEG is
        // far smaller than the full-res wide/zoom visuals), which is a much safer fallback than
        // blindly trusting the first-reported file.
        val thermal = all.firstOrNull { isThermalName(it.fileName) }
            ?: all.minByOrNull { it.fileSize }
            ?: newFiles.first()
        // Wide visual (RGB) and zoom siblings, distinguished by lens suffix.
        val visual = all.firstOrNull { isWideName(it.fileName) && it.fileName != thermal.fileName }
        val zoom = all.firstOrNull { isZoomName(it.fileName) && it.fileName != thermal.fileName }

        if (visual == null && zoom == null) {
            Log.w(TAG, "No wide/zoom siblings found — payload may be set to store infrared only")
        } else {
            Log.i(TAG, "Paired thermal=${thermal.fileName} (${thermal.fileSize}B) " +
                "visual=${visual?.fileName} (${visual?.fileSize}B) zoom=${zoom?.fileName} (${zoom?.fileSize}B)")
        }
        return ThermalCapture(thermal, visual, zoom)
    }

    private fun jsonName(name: String?) = if (name == null) "null" else "\"$name\""

    // Returns null if the shutter produced no thermal file. Blocking, call from a worker thread.
    fun captureThermal(mediaVM: MediaVM): String? {
        val capture = takeThermalAndVisual(mediaVM)
        if (capture.thermal == null) return null
        return "{\"thermal\":${jsonName(capture.thermal.fileName)}," +
            "\"wide\":${jsonName(capture.visual?.fileName)}," +
            "\"zoom\":${jsonName(capture.zoom?.fileName)}}"
    }
    // Trip one shutter on the payload and return ALL media files it produced (thermal R-JPEG plus,
    // when visible storage is enabled, the wide/zoom visual photo). Blocking, call from a worker
    // thread. mediaVM must be init with SD card storage and the LEFT_OR_MAIN component index
    // (done in the host activity's onCreate).
    private fun captureNewMediaFiles(mediaVM: MediaVM): List<MediaFile> {
        try {
            setupNewMediaListener()
            val keyBaseline = (keyNewlyGeneratedMediaFile.get() ?: latestGeneratedMediaInfo)?.index
            val listBaseline = mediaVM.mediaFileListData.value?.data?.maxOfOrNull { it.fileIndex }
            val baselineIndex = listOfNotNull(keyBaseline, listBaseline).maxOrNull()
            latestGeneratedMediaInfo = null
            Log.i(TAG, "Pre-shutter baseline index=$baselineIndex (key=$keyBaseline, list=$listBaseline)")

            // Arm event collection so every file this shutter writes is queued for the resolve below.
            mediaEventQueue.clear()
            collectingEvents = true

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
                return emptyList()
            }
            if (photoError != null) return emptyList()

            val overallDeadline = System.currentTimeMillis() + CAPTURE_OVERALL_TIMEOUT_MS

            // ---- Phase 1: wait on the camera's push events — ZERO list pulls -------------------
            // Each file the shutter writes fires KeyNewlyGeneratedMediaFile (queued above). The push
            // carries the index and lens (dcf_type), so we learn what landed without touching the
            // media list. Stop, fastest-first:
            //   COMPLETE  — thermal + wide + zoom lenses all reported; nothing more can arrive.
            //   SETTLED   — no new event for EVENT_SETTLE_MS, at least the thermal landed (covers
            //               payloads/configs that store fewer lenses).
            // If NO event arrives within EVENT_FIRST_TIMEOUT_MS the key lagged/dropped (the known
            // rapid-fire failure mode) — bail to the pull loop in phase 3.
            val seenIndices = HashSet<Int>()
            var thermalSeen = false; var wideSeen = false; var zoomSeen = false
            val phaseStart = System.currentTimeMillis()
            while (System.currentTimeMillis() < overallDeadline) {
                val waitMs = (overallDeadline - System.currentTimeMillis()).coerceAtMost(EVENT_SETTLE_MS)
                val ev = mediaEventQueue.poll(waitMs, TimeUnit.MILLISECONDS)
                if (ev != null) {
                    val idx = ev.index ?: continue
                    if (baselineIndex == null || idx > baselineIndex) {
                        seenIndices.add(idx)
                        when {
                            isThermalLens(ev.dcf_type) -> thermalSeen = true
                            isWideLens(ev.dcf_type) -> wideSeen = true
                            isZoomLens(ev.dcf_type) -> zoomSeen = true
                        }
                    }
                    if (thermalSeen && wideSeen && zoomSeen) break          // COMPLETE
                } else if (thermalSeen || seenIndices.isNotEmpty()) {
                    break                                                   // SETTLED (event gap elapsed)
                } else if (System.currentTimeMillis() - phaseStart >= EVENT_FIRST_TIMEOUT_MS) {
                    break                                                   // nothing landed -> fall back
                }
            }
            collectingEvents = false
            Log.i(TAG, "Events after shutter in ${System.currentTimeMillis() - phaseStart}ms: " +
                "indices=$seenIndices (thermal=$thermalSeen wide=$wideSeen zoom=$zoomSeen)")

            // ---- Phase 2: ONE list pull to materialise handles for the reported files ----------
            // The events told us WHAT landed; this single pull turns those indices into downloadable
            // MediaFile handles. Reuse the robust grouping (DCF base name OR above-baseline OR a seen
            // index) so a dropped event can't silently drop a sibling.
            if (seenIndices.isNotEmpty()) {
                val data = if (narrowPullSupported)
                    mediaVM.pullAndAwait(MEDIA_PULL_TIMEOUT_MS, CAPTURE_PULL_COUNT, FileListRequestTimeOrderType.NEW_FIRST)
                else
                    mediaVM.pullAndAwait(MEDIA_PULL_TIMEOUT_MS)
                val anchor = data.filter { baselineIndex == null || it.fileIndex > baselineIndex }
                    .maxByOrNull { it.fileIndex }
                    ?: data.filter { it.fileIndex in seenIndices }.maxByOrNull { it.fileIndex }
                if (anchor != null) {
                    val groupBase = lensGroupBase(anchor.fileName)
                    val group = data.filter {
                        lensGroupBase(it.fileName) == groupBase ||
                            (baselineIndex != null && it.fileIndex > baselineIndex) ||
                            it.fileIndex in seenIndices
                    }.distinctBy { it.fileName }.ifEmpty { listOf(anchor) }
                    Log.i(TAG, "Resolved ${group.size} file(s) above baseline $baselineIndex in one pull: " +
                        group.joinToString { "${it.fileName}#${it.fileIndex}" })
                    return group
                }
                Log.w(TAG, "Single pull found no handle for seen indices $seenIndices; falling back to pull loop")
            }

            // ---- Phase 3: fallback poll-pull loop (key lagged/dropped, or handle not yet listed) -
            // Battle-tested under rapid fire. Two exits, fastest-first:
            //   COMPLETE  — all three exposed lenses present; return immediately.
            //   QUIESCENT — file set unchanged across two refreshes, thermal present.
            var bestGroup: List<MediaFile> = emptyList()
            var prevGroupNames: Set<String> = emptySet()
            var emptyNarrowPulls = 0

            while (System.currentTimeMillis() < overallDeadline) {
                val narrow = narrowPullSupported
                val data = if (narrow)
                    mediaVM.pullAndAwait(MEDIA_PULL_TIMEOUT_MS, CAPTURE_PULL_COUNT, FileListRequestTimeOrderType.NEW_FIRST)
                else
                    mediaVM.pullAndAwait(MEDIA_PULL_TIMEOUT_MS)

                val anchor = data.filter { baselineIndex == null || it.fileIndex > baselineIndex }
                    .maxByOrNull { it.fileIndex }

                // Detect a firmware that ignores NEW_FIRST: a FULL newest-window whose max index is
                // strictly BELOW the baseline can only be the oldest files (wrong order). A shot that
                // simply hasn't landed yet leaves max == baseline, so this never misfires on a slow
                // write. After a few such pulls, drop to full pulls for the rest of the session.
                if (narrow && baselineIndex != null && data.size >= CAPTURE_PULL_COUNT &&
                    (data.maxOfOrNull { it.fileIndex } ?: Int.MAX_VALUE) < baselineIndex) {
                    if (++emptyNarrowPulls >= NARROW_PULL_FALLBACK_TRIES) {
                        narrowPullSupported = false
                        Log.w(TAG, "Narrow pull returned only files older than baseline $baselineIndex " +
                            "($emptyNarrowPulls times); firmware ignores NEW_FIRST — using full pulls for the session")
                    }
                }
                if (anchor != null) {
                    // This shutter's lens files, identified two independent ways and unioned:
                    //   1. sharing the anchor's DCF base name (the co-exposed _T/_W/_Z siblings), and
                    //   2. newer than the pre-shutter baseline (captures are serial).
                    // Both are index-encoding agnostic, so this works across the H20T/H20N/H30T.
                    val groupBase = lensGroupBase(anchor.fileName)
                    val byBase = data.filter { lensGroupBase(it.fileName) == groupBase }
                    val byBaseline = if (baselineIndex != null) data.filter { it.fileIndex > baselineIndex } else emptyList()
                    val group = (byBase + byBaseline).distinctBy { it.fileName }.ifEmpty { listOf(anchor) }

                    if (group.size > bestGroup.size) bestGroup = group
                    val names = bestGroup.map { it.fileName }.toSet()
                    val hasThermal = bestGroup.any { isThermalName(it.fileName) }

                    // (1) Complete: all three exposed lenses present — return immediately, no extra pull.
                    val complete = hasThermal &&
                        bestGroup.any { isWideName(it.fileName) } &&
                        bestGroup.any { isZoomName(it.fileName) }
                    // (2) Quiescent: this refresh added nothing new to the group, thermal present.
                    if (complete || (hasThermal && names == prevGroupNames)) {
                        Log.i(TAG, "Resolved ${bestGroup.size} file(s) above baseline $baselineIndex " +
                            "(${if (complete) "complete" else "settled"}): " +
                            bestGroup.joinToString { "${it.fileName}#${it.fileIndex}" })
                        return bestGroup
                    }
                    prevGroupNames = names
                }
            }

            // Hit the safety cap before the group settled. Return the best (largest) group seen so the
            // caller still gets whatever lenses did surface; empty only if nothing ever matched.
            if (bestGroup.isNotEmpty()) {
                Log.w(TAG, "Group above baseline $baselineIndex did not settle with thermal; returning " +
                    "best-effort ${bestGroup.size} file(s): " + bestGroup.joinToString { "${it.fileName}#${it.fileIndex}" })
                return bestGroup
            }
            Log.e(TAG, "No new files surfaced above baseline $baselineIndex after shutter")
            return emptyList()
        } catch (e: Exception) {
            Log.e(TAG, "Error taking thermal image: ${e.message}", e)
            return emptyList()
        } finally {
            // Always disarm collection — also covers the pre-phase-1 early returns (shutter timeout/fail).
            collectingEvents = false
        }
    }

    // Download a MediaFile from the camera straight into memory (no disk round-trip) and return
    // its bytes, or null on failure. Blocking. The DJI-link download is the bottleneck; skipping
    // the write-to-flash-then-read-back saves tens of MB of needless I/O per capture.
    private fun downloadToBytes(photoFile: MediaFile): ByteArray? {
        Log.i(TAG, "Downloading ${photoFile.fileName} (${photoFile.fileSize} bytes) to memory")
        // Pre-size the buffer to the known file size to avoid repeated array growth/copies.
        val initialCapacity = photoFile.fileSize.takeIf { it in 1..Int.MAX_VALUE.toLong() }?.toInt() ?: (1 shl 20)
        val buffer = java.io.ByteArrayOutputStream(initialCapacity)

        val downloadLatch = CountDownLatch(1)
        var downloadError: String? = null

        photoFile.pullOriginalMediaFileFromCamera(0L, object : MediaFileDownloadListener {
            override fun onStart() { /* no-op */ }
            override fun onProgress(total: Long, current: Long) { /* no-op */ }
            override fun onRealtimeDataUpdate(data: ByteArray, position: Long) {
                // Callbacks arrive in order from offset 0, so a plain append reconstructs the file.
                buffer.write(data, 0, data.size)
            }
            override fun onFinish() {
                downloadLatch.countDown()
            }
            override fun onFailure(error: IDJIError?) {
                downloadError = error?.description() ?: "Unknown download error"
                Log.e(TAG, "Download failed for ${photoFile.fileName}: $downloadError")
                downloadLatch.countDown()
            }
        })

        if (!downloadLatch.await(120, TimeUnit.SECONDS)) {
            Log.e(TAG, "Timeout downloading ${photoFile.fileName}")
            return null
        }
        if (downloadError != null) return null
        val bytes = buffer.toByteArray()
        if (bytes.isEmpty()) {
            Log.e(TAG, "Downloaded ${photoFile.fileName} is empty")
            return null
        }
        Log.i(TAG, "Downloaded ${photoFile.fileName} (${bytes.size} bytes) to memory")
        return bytes
    }

    // Target short side (px) for downscaled zoom images, and JPEG quality of the re-encode.
    private const val ZOOM_TARGET_SHORT_SIDE = 1080
    private const val ZOOM_JPEG_QUALITY = 85

    // Decode a JPEG, downscale so its SHORTER side is ~1080 px (aspect preserved, never upscaled),
    // and re-encode to JPEG. The re-encode discards all EXIF/metadata by design. Returns the
    // original bytes unchanged if the image can't be decoded or is already <= the target.
    private fun downscaleZoomTo1080p(jpeg: ByteArray, label: String): ByteArray {
        // Pass 1: read dimensions only, no pixel allocation.
        val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
        BitmapFactory.decodeByteArray(jpeg, 0, jpeg.size, bounds)
        val srcW = bounds.outWidth
        val srcH = bounds.outHeight
        if (srcW <= 0 || srcH <= 0) {
            Log.w(TAG, "Zoom downscale: undecodable $label, sending original")
            return jpeg
        }
        val shortSide = minOf(srcW, srcH)
        if (shortSide <= ZOOM_TARGET_SHORT_SIDE) return jpeg  // already small enough

        // Pass 2: power-of-2 subsample to just above target (bounds peak memory), then exact scale.
        var sample = 1
        while (shortSide / (sample * 2) >= ZOOM_TARGET_SHORT_SIDE) sample *= 2
        val decoded = BitmapFactory.decodeByteArray(
            jpeg, 0, jpeg.size, BitmapFactory.Options().apply { inSampleSize = sample }
        ) ?: run {
            Log.w(TAG, "Zoom downscale: decode failed $label, sending original")
            return jpeg
        }

        val scale = ZOOM_TARGET_SHORT_SIDE.toFloat() / minOf(decoded.width, decoded.height)
        val dstW = (decoded.width * scale).roundToInt()
        val dstH = (decoded.height * scale).roundToInt()
        val scaled = Bitmap.createScaledBitmap(decoded, dstW, dstH, true)
        if (scaled !== decoded) decoded.recycle()

        val baos = java.io.ByteArrayOutputStream()
        scaled.compress(Bitmap.CompressFormat.JPEG, ZOOM_JPEG_QUALITY, baos)
        scaled.recycle()
        val out = baos.toByteArray()
        Log.i(TAG, "Zoom downscale $label: ${srcW}x$srcH -> ${dstW}x$dstH (${jpeg.size} -> ${out.size} bytes)")
        return out
    }

    // ==================== SD-card media access (robust, source-of-truth path) ====================
    //
    // Downloads resolve by FILENAME against the camera's own media list (mediaVM.mediaFileListData),
    // the real index of everything on the card. captureThermal returns the per-lens filenames; the
    // client downloads any of them — or any other file on the card — by name. There is no server-side
    // capture cache to bound, so every photo ever taken stays reachable even under long bursts of
    // consecutive captures.

    // Resolve a MediaFile by its filename from the live media list. Searches top-level entries and
    // their subMediaFile variants. The cached snapshot can lag a just-taken shot, so if the name is
    // absent we refresh (event-driven, blocking on each pull's completion) and retry until the
    // overall safety cap. Returns null if it never surfaces.
    private fun findMediaFile(mediaVM: MediaVM, fileName: String): MediaFile? {
        setupNewMediaListener()
        fun search(data: List<MediaFile>?): MediaFile? {
            data ?: return null
            for (f in data) {
                if (f.fileName == fileName) return f
                f.subMediaFile?.forEach { sub -> if (sub?.fileName == fileName) return sub }
            }
            return null
        }
        // Cheap path: maybe it's already in the cached snapshot.
        search(mediaVM.mediaFileListData.value?.data)?.let { return it }
        val deadline = System.currentTimeMillis() + CAPTURE_OVERALL_TIMEOUT_MS
        while (System.currentTimeMillis() < deadline) {
            search(mediaVM.pullAndAwait(MEDIA_PULL_TIMEOUT_MS))?.let { return it }
        }
        return null
    }

    // Flatten a media-list snapshot into a name-keyed map: top-level entries plus their
    // subMediaFile variants, deduped by filename so every downloadable file appears once.
    private fun flattenMedia(data: List<MediaFile>?): LinkedHashMap<String, MediaFile> {
        val files = LinkedHashMap<String, MediaFile>()
        data?.forEach { f ->
            f.fileName?.let { files.putIfAbsent(it, f) }
            f.subMediaFile?.forEach { sub -> sub?.fileName?.let { files.putIfAbsent(it, sub) } }
        }
        return files
    }

    // Pull the full SD-card file list and return it as a JSON array describing every file, so a
    // client can browse and download ANY picture by name (not only the handful from recent
    // shutters). Blocking, call from a worker thread.
    //
    // mediaFileListData refreshes ONLY when a pull completes; the SDK never rescans the card on its
    // own. The cached snapshot is therefore usually stale, so we refresh (event-driven — each
    // pullAndAwait blocks on the SDK completion signal) until the file count is QUIESCENT, i.e. it
    // holds steady across two consecutive refreshes, meaning no more files are arriving.
    fun listAllMedia(mediaVM: MediaVM): String {
        setupNewMediaListener()

        val deadline = System.currentTimeMillis() + CAPTURE_OVERALL_TIMEOUT_MS
        var files = flattenMedia(mediaVM.mediaFileListData.value?.data)
        while (System.currentTimeMillis() < deadline) {
            val next = flattenMedia(mediaVM.pullAndAwait(MEDIA_PULL_TIMEOUT_MS))
            // Quiescent: count held steady across this refresh and we have at least one file.
            if (next.size == files.size && next.isNotEmpty()) {
                files = next
                break
            }
            files = next
        }

        val items = files.values.joinToString(",") { f ->
            "{\"name\":${jsonName(f.fileName)},\"index\":${f.fileIndex}," +
                "\"size\":${f.fileSize},\"type\":${jsonName(f.fileType?.name)}}"
        }
        Log.i(TAG, "Listed ${files.size} media file(s) on SD card")
        return "{\"count\":${files.size},\"files\":[$items]}"
    }

    // Resolve a file by name from the live SD-card list and stream it back as image/jpeg. Works for
    // any file on the card. Blocking, call from a worker thread.
    fun sendMediaFileByName(mediaVM: MediaVM, fileName: String, outputStream: OutputStream) {
        val mediaFile = findMediaFile(mediaVM, fileName)
        if (mediaFile == null) {
            sendErrorResponse(outputStream, "File not found on SD card: $fileName")
            return
        }
        streamMediaFile(mediaFile, fileName, outputStream)
    }

    // Download a MediaFile and write it back over the socket as an image/jpeg response, or an error
    // response on failure.
    private fun streamMediaFile(mediaFile: MediaFile, label: String, outputStream: OutputStream) {
        try {
            val original = downloadToBytes(mediaFile)
            if (original == null) {
                sendErrorResponse(outputStream, "Failed to download $label")
                return
            }
            // Zoom lens shots are full-res (very large). Downscale to 1080p before sending; the
            // re-encode also strips all EXIF/metadata, which we don't want forwarded. Other lenses
            // (wide/thermal) are sent untouched.
            val bytes = if (isZoomName(mediaFile.fileName)) {
                downscaleZoomTo1080p(original, mediaFile.fileName)
            } else {
                original
            }
            val headers = StringBuilder().apply {
                append("HTTP/1.1 200 OK\r\n")
                append("Content-Type: image/jpeg\r\n")
                append("Content-Length: ${bytes.size}\r\n")
                append("Content-Disposition: attachment; filename=\"${mediaFile.fileName}\"\r\n")
                append("Access-Control-Allow-Origin: *\r\n")
                append("\r\n")
            }
            outputStream.write(headers.toString().toByteArray())
            outputStream.write(bytes)
            outputStream.flush()

            Log.i(TAG, "Sent ${mediaFile.fileName} (${bytes.size} bytes)")
            mainHandler.post {
                ToastUtils.showToast("Sent ${mediaFile.fileName} (${bytes.size / 1024} KB)")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error sending ${mediaFile.fileName}: ${e.message}", e)
            try {
                sendErrorResponse(outputStream, "Error sending $label: ${e.message}")
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

    // ==================== Payload drop (release servo) ====================

    // The PayloadIndexType the widget VM is currently wired to. initListener() must run once
    // before setWidgetValue() (it sets a lateinit field in the VM); we re-register only when
    // the port changes so repeated drops don't stack duplicate payload listeners.
    @Volatile
    private var dropListenerIndex: PayloadIndexType? = null

    // Fire the payload release on [indexType], reproducing the manual main-interface widget
    // sequence: arm the unlock SWITCH, pulse the release BUTTON on then off, then re-lock.
    // The 300 ms gaps let the payload firmware register each discrete widget change.
    //
    // [armSwitchIndex]/[releaseButtonIndex] are payload widgetIndex values supplied by the
    // active DroneControlProfile. The original working PORT_3 logic (M400/M3E/M350) is
    // SWITCH 0 + BUTTON 1; the M300 SkyPort payload (TH4) uses Unlock SWITCH 3 + All_Down
    // BUTTON 5.
    //
    // Blocking, call from a worker thread. payloadWidgetVM must be obtained from the host
    // activity via ViewModelProvider (created on the main thread).
    fun dropPayload(
        payloadWidgetVM: PayloadWidgetVM,
        indexType: PayloadIndexType,
        armSwitchIndex: Int = 0,
        releaseButtonIndex: Int = 2
    ): Boolean {
        return try {
            if (dropListenerIndex != indexType) {
                payloadWidgetVM.initListener(indexType)
                dropListenerIndex = indexType
            }

            val armSwitch = WidgetValue().apply {
                type = WidgetType.SWITCH
                index = armSwitchIndex
                value = 1
            }
            payloadWidgetVM.setWidgetValue(armSwitch)
            Thread.sleep(300)

            val releaseButton = WidgetValue().apply {
                type = WidgetType.BUTTON
                index = releaseButtonIndex
                value = 1
            }
            payloadWidgetVM.setWidgetValue(releaseButton)
            Thread.sleep(300)

            releaseButton.value = 0
            payloadWidgetVM.setWidgetValue(releaseButton)
            Thread.sleep(300)

            armSwitch.value = 0
            payloadWidgetVM.setWidgetValue(armSwitch)

            Log.i(TAG, "Payload drop sequence sent on $indexType (switch=$armSwitchIndex, button=$releaseButtonIndex)")
            true
        } catch (e: Exception) {
            Log.e(TAG, "Payload drop failed: ${e.message}", e)
            false
        }
    }

    // Cancel payload key listeners. Call from the host activity's onDestroy.
    fun destroy() {
        KeyManager.getInstance().cancelListen(this)
        lrfListenerRegistered = false
        lrfInfo = null
        newMediaListenerRegistered = false
        latestGeneratedMediaInfo = null
        mediaWarmedUp = false
        dropListenerIndex = null
    }
}
