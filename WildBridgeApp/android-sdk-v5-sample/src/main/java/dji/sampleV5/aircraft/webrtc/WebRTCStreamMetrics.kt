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
        val errorLabel = if (processingErrors > 0) " err $processingErrors" else ""
        val recoveryLabel = if (recoveryCount > 0) " fix $recoveryCount" else ""
        return "WEBRTC $status out $resolutionLabel src $source fps ${outputFps.format1()}/${targetFps} drop ${droppedFps.format1()} proc ${averageFrameProcessingMs.format1()}ms clients $observerCount$errorLabel$recoveryLabel"
    }

    private fun Double.format1(): String = String.format(java.util.Locale.US, "%.1f", this)
}