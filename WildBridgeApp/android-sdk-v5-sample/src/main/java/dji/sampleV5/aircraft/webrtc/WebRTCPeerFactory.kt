package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.util.Log
import org.webrtc.DefaultVideoDecoderFactory
import org.webrtc.DefaultVideoEncoderFactory
import org.webrtc.EglBase
import org.webrtc.PeerConnectionFactory

object WebRTCPeerFactory {
    private const val TAG = "WebRTCPeerFactory"

    @Volatile private var factory: PeerConnectionFactory? = null
    private var eglBase: EglBase? = null
    private val factoryLock = Any()

    fun getEglBase(): EglBase {
        synchronized(factoryLock) {
            if (eglBase == null) {
                eglBase = EglBase.create()
            }
            return eglBase!!
        }
    }

    fun getFactory(context: Context): PeerConnectionFactory {
        synchronized(factoryLock) {
            if (factory == null) {
                initializeFactory(context)
            }
            return factory!!
        }
    }

    private fun initializeFactory(context: Context) {
        val initOptions = PeerConnectionFactory.InitializationOptions.builder(context)
            .setEnableInternalTracer(true)
            .setFieldTrials("WebRTC-H264HighProfile/Enabled/")
            .createInitializationOptions()
        PeerConnectionFactory.initialize(initOptions)

        val rootEglBase = getEglBase()

        factory = PeerConnectionFactory.builder()
            .setVideoDecoderFactory(DefaultVideoDecoderFactory(rootEglBase.eglBaseContext))
            .setVideoEncoderFactory(
                DefaultVideoEncoderFactory(
                    rootEglBase.eglBaseContext,
                    false,
                    true
                )
            )
            .setOptions(PeerConnectionFactory.Options())
            .createPeerConnectionFactory()

        Log.d(TAG, "PeerConnectionFactory initialized")
    }
}