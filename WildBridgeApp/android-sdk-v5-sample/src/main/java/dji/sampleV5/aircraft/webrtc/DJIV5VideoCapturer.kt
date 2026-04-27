package dji.sampleV5.aircraft.webrtc

import android.content.Context
import android.util.Log
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import org.webrtc.CapturerObserver
import org.webrtc.NV21Buffer
import org.webrtc.SurfaceTextureHelper
import org.webrtc.VideoCapturer
import org.webrtc.VideoFrame
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

/**
 * VideoCapturer implementation for DJI SDK V5 that captures frames
 * from the drone's camera using CameraStreamManager.
 * 
 * This adapter captures YUV frames from the DJI camera and converts them
 * to WebRTC VideoFrames for transmission via WebRTC.
 * 
 * Also captures synchronized telemetry metadata for each frame.
 */
class DJIV5VideoCapturer(
    private val cameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN,
    @Volatile var targetWidth: Int = FULL_HD_WIDTH,
    @Volatile var targetHeight: Int = FULL_HD_HEIGHT,
    private val scaleToTarget: Boolean = true,
    private val droneName: String = "drone_1"
) : VideoCapturer {

    companion object {
        private const val TAG = "DJIV5VideoCapturer"
        
        // Resolution presets
        const val FULL_HD_WIDTH = 1920
        const val FULL_HD_HEIGHT = 1080
        const val HD_WIDTH = 1280
        const val HD_HEIGHT = 720
    }

    private var capturerObserver: CapturerObserver? = null
    private var surfaceTextureHelper: SurfaceTextureHelper? = null
    private val isCapturing = AtomicBoolean(false)
    @Volatile private var targetFps = 30
    @Volatile private var frameIntervalNs = 1_000_000_000L / 30L
    private var lastSourceWidth = 0
    private var lastSourceHeight = 0
    private val lastSentTimestampNs = AtomicLong(0L)
    
    // Frame counter for metadata synchronization
    private val frameCounter = AtomicLong(0)
    
    // Listener for frame metadata (called for each frame with synchronized telemetry)
    var metadataListener: FrameMetadataListener? = null
    
    interface FrameMetadataListener {
        fun onFrameMetadata(metadata: FrameMetadata)
    }
    
    private val cameraStreamManager: ICameraStreamManager by lazy {
        MediaDataCenter.getInstance().cameraStreamManager
    }

    private val frameListener = object : ICameraStreamManager.CameraFrameListener {
        override fun onFrame(
            frameData: ByteArray,
            offset: Int,
            length: Int,
            width: Int,
            height: Int,
            format: ICameraStreamManager.FrameFormat
        ) {
            if (!isCapturing.get() || capturerObserver == null) return

            try {
                val timestampNs = System.nanoTime()

                // Cap output FPS by dropping frames that arrive too soon.
                val previousSent = lastSentTimestampNs.get()
                if (previousSent != 0L && (timestampNs - previousSent) < frameIntervalNs) {
                    return
                }
                lastSentTimestampNs.set(timestampNs)

                // Log source resolution changes
                if (width != lastSourceWidth || height != lastSourceHeight) {
                    lastSourceWidth = width
                    lastSourceHeight = height
                    Log.d(TAG, "Source: ${width}x${height}, Target: ${targetWidth}x${targetHeight}, Scale: $scaleToTarget")
                }

                val frameNumber = frameCounter.incrementAndGet()
                
                // Determine output dimensions
                val outputWidth = if (scaleToTarget) targetWidth else width
                val outputHeight = if (scaleToTarget) targetHeight else height
                
                // Capture synchronized telemetry metadata at this exact moment
                metadataListener?.let { listener ->
                    val metadata = TelemetryProvider.captureMetadata(
                        frameNumber = frameNumber,
                        timestampNs = timestampNs,
                        frameWidth = outputWidth,
                        frameHeight = outputHeight,
                        droneName = droneName
                    )
                    Log.v(TAG, "Captured metadata for frame $frameNumber: lat=${metadata.latitude}, lon=${metadata.longitude}, battery=${metadata.batteryPercent}%")
                    listener.onFrameMetadata(metadata)
                }
                
                // Create NV21 buffer from the frame data
                val buffer = NV21Buffer(
                    frameData,
                    width,
                    height,
                    null
                )
                
                // Scale to target resolution if enabled and dimensions differ
                val outputBuffer = if (scaleToTarget && (width != targetWidth || height != targetHeight)) {
                    buffer.cropAndScale(
                        0, 0, width, height,  // Use full source frame
                        targetWidth, targetHeight  // Scale to target
                    )
                } else {
                    buffer
                }
                
                val videoFrame = VideoFrame(outputBuffer, 0, timestampNs)
                capturerObserver?.onFrameCaptured(videoFrame)
                videoFrame.release()
                
            } catch (e: Exception) {
                Log.e(TAG, "Error processing frame: ${e.message}", e)
            }
        }
    }

    override fun initialize(
        surfaceTextureHelper: SurfaceTextureHelper?,
        applicationContext: Context,
        capturerObserver: CapturerObserver
    ) {
        Log.d(TAG, "Initializing DJIV5VideoCapturer for camera: $cameraIndex, target: ${targetWidth}x${targetHeight}")
        this.surfaceTextureHelper = surfaceTextureHelper
        this.capturerObserver = capturerObserver
    }

    override fun startCapture(width: Int, height: Int, framerate: Int) {
        targetFps = framerate.coerceAtLeast(1)
        frameIntervalNs = 1_000_000_000L / targetFps.toLong()
        lastSentTimestampNs.set(0L)
        Log.d(TAG, "Starting capture: ${targetWidth}x${targetHeight}@${targetFps}fps (scale=$scaleToTarget)")
        
        if (isCapturing.compareAndSet(false, true)) {
            // Register frame listener with NV21 format (compatible with WebRTC's NV21Buffer)
            cameraStreamManager.addFrameListener(
                cameraIndex,
                ICameraStreamManager.FrameFormat.NV21,
                frameListener
            )
            
            capturerObserver?.onCapturerStarted(true)
            Log.d(TAG, "Capture started successfully")
        }
    }

    override fun stopCapture() {
        Log.d(TAG, "Stopping capture")
        
        if (isCapturing.compareAndSet(true, false)) {
            cameraStreamManager.removeFrameListener(frameListener)
            capturerObserver?.onCapturerStopped()
            Log.d(TAG, "Capture stopped")
        }
    }

    /**
     * Change the target resolution on-the-fly. Takes effect on the next frame.
     */
    fun changeResolution(width: Int, height: Int) {
        Log.d(TAG, "Changing target resolution: ${targetWidth}x${targetHeight} -> ${width}x${height}")
        targetWidth = width
        targetHeight = height
    }

    override fun changeCaptureFormat(width: Int, height: Int, framerate: Int) {
        Log.d(TAG, "Change capture format requested: ${width}x${height}@${framerate}fps")
        changeResolution(width, height)
        targetFps = framerate.coerceAtLeast(1)
        frameIntervalNs = 1_000_000_000L / targetFps.toLong()
    }

    override fun dispose() {
        Log.d(TAG, "Disposing DJIV5VideoCapturer")
        stopCapture()
        capturerObserver = null
        surfaceTextureHelper = null
    }

    override fun isScreencast(): Boolean = false
}
