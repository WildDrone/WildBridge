package dji.sampleV5.aircraft.webrtc

data class WebRTCStreamMetrics(
    val sourceWidth: Int = 0,
    val sourceHeight: Int = 0,
    val outputWidth: Int = 0,
    val outputHeight: Int = 0,
    val requestedWidth: Int = 0,
    val requestedHeight: Int = 0,
    val targetFps: Int = 0,
    val inputFps: Double = 0.0,
    val outputFps: Double = 0.0,
    val droppedFps: Double = 0.0,
    val averageFrameProcessingMs: Double = 0.0,
    val totalFrames: Long = 0,
    val totalDroppedFrames: Long = 0,
    val processingErrors: Long = 0,
    val observerCount: Int = 0,
    val activeCamera: String = "unknown",
    val status: String = "idle",
    val configuredFps: Int = 0,
    val saturationState: String = "ok",
    val scaleMode: String = "fixed",
    val recoveryCount: Int = 0,
    val lastError: String? = null
) {
    val resolutionLabel: String
        get() = if (outputWidth > 0 && outputHeight > 0) {
            "${outputWidth}x${outputHeight}"
        } else {
            "waiting"
        }

    fun compactLabel(): String {
        val source = if (sourceWidth > 0 && sourceHeight > 0) "${sourceWidth}x${sourceHeight}" else "waiting"
        val requested = if (requestedWidth > 0 && requestedHeight > 0) "${requestedWidth}x${requestedHeight}" else "native"
        val saturationLabel = if (saturationState != "ok") " sat $saturationState" else ""
        val fpsLabel = if (configuredFps > 0 && configuredFps != targetFps) {
            "${outputFps.format1()}/${targetFps} cfg $configuredFps"
        } else {
            "${outputFps.format1()}/${targetFps}"
        }
        val errorLabel = if (processingErrors > 0) " err $processingErrors" else ""
        val recoveryLabel = if (recoveryCount > 0) " fix $recoveryCount" else ""
        return "WEBRTC $status$saturationLabel out $resolutionLabel req $requested src $source fps $fpsLabel drop ${droppedFps.format1()} resize ${averageFrameProcessingMs.format1()}ms scale $scaleMode clients $observerCount$errorLabel$recoveryLabel"
    }

    private fun Double.format1(): String = String.format(java.util.Locale.US, "%.1f", this)
}