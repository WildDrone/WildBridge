package dji.sampleV5.aircraft.webrtc

import android.util.Log
import org.webrtc.SdpObserver
import org.webrtc.SessionDescription

/**
 * Simple SdpObserver implementation with logging and optional callbacks.
 */
open class SimpleSdpObserver(
    private val tag: String = "SimpleSdpObserver",
    private val onSuccess: ((SessionDescription?) -> Unit)? = null,
    private val onFailure: ((String) -> Unit)? = null
) : SdpObserver {

    override fun onCreateSuccess(sessionDescription: SessionDescription?) {
        Log.d(tag, "onCreateSuccess: ${sessionDescription?.type}")
        onSuccess?.invoke(sessionDescription)
    }

    override fun onSetSuccess() {
        Log.d(tag, "onSetSuccess")
    }

    override fun onCreateFailure(error: String?) {
        Log.e(tag, "onCreateFailure: $error")
        onFailure?.invoke(error ?: "Unknown error")
    }

    override fun onSetFailure(error: String?) {
        Log.e(tag, "onSetFailure: $error")
        onFailure?.invoke(error ?: "Unknown error")
    }
}
