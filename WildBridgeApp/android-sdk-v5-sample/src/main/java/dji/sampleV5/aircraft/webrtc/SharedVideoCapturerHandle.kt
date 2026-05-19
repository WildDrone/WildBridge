package dji.sampleV5.aircraft.webrtc

import android.content.Context
import org.webrtc.CapturerObserver
import org.webrtc.SurfaceTextureHelper
import org.webrtc.VideoCapturer

/**
 * Lightweight per-client [VideoCapturer] proxy that delegates to a
 * [SharedDJIFrameSource].  Each [WebRTCClient] receives its own handle
 * while the expensive DJI frame listener and NV21 scaling run only once
 * inside the shared source.
 */
class SharedVideoCapturerHandle(
    private val clientId: String,
    private val source: SharedDJIFrameSource
) : VideoCapturer {

    var metadataListener: DJIV5VideoCapturer.FrameMetadataListener? = null
        set(value) {
            field = value
            source.setMetadataListener(clientId, value)
        }

    override fun initialize(
        surfaceTextureHelper: SurfaceTextureHelper?,
        applicationContext: Context,
        capturerObserver: CapturerObserver
    ) {
        source.registerObserver(clientId, capturerObserver)
    }

    override fun startCapture(width: Int, height: Int, framerate: Int) {
        source.startClient(clientId, width, height, framerate)
    }

    override fun stopCapture() {
        source.stopClient(clientId)
    }

    fun changeResolution(width: Int, height: Int) {
        source.changeResolution(width, height)
    }

    fun changeFrameRate(fps: Int) {
        source.changeFrameRate(fps)
    }

    override fun changeCaptureFormat(width: Int, height: Int, framerate: Int) {
        source.changeResolution(width, height)
        source.changeFrameRate(framerate)
    }

    override fun dispose() {
        source.stopClient(clientId)
    }

    override fun isScreencast(): Boolean = false
}
