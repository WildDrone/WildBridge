package dji.sampleV5.aircraft.pages

import android.annotation.SuppressLint
import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.RadioGroup
import android.widget.TextView
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sampleV5.aircraft.webrtc.WebRTCMediaOptions
import dji.sampleV5.aircraft.webrtc.WebRTCStreamMetrics
import dji.sampleV5.aircraft.webrtc.WebRTCStreamer
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager

/**
 * Fragment for WebRTC streaming of drone video feed.
 * Allows starting a WebRTC server for local network clients to connect and view the drone's camera feed.
 */
class WebRTCStreamFragment : DJIFragment() {

    companion object {
        private const val DEFAULT_PORT = 8081
    }

    private val cameraStreamManager: ICameraStreamManager by lazy {
        MediaDataCenter.getInstance().cameraStreamManager
    }

    // UI elements
    private lateinit var btnStartServer: Button
    private lateinit var btnStopServer: Button
    private lateinit var rgCamera: RadioGroup
    private lateinit var tvServerStatus: TextView
    private lateinit var tvServerIp: TextView
    private lateinit var tvServerPort: TextView
    private lateinit var tvClientCount: TextView
    private lateinit var tvConnectionUrl: TextView
    private lateinit var tvStreamMetrics: TextView
    private lateinit var btnCopyUrl: Button
    private lateinit var svCameraPreview: SurfaceView
    private lateinit var btnShowPreview: Button
    private lateinit var btnHidePreview: Button
    private lateinit var tvErrorMessage: TextView

    // WebRTC
    private var webRTCStreamer: WebRTCStreamer? = null
    private var selectedCameraIndex: ComponentIndexType = ComponentIndexType.LEFT_OR_MAIN
    private var selectedOptions: WebRTCMediaOptions = WebRTCMediaOptions.fullHD()

    // Camera preview
    private var previewSurface: Surface? = null
    private var previewWidth = -1
    private var previewHeight = -1
    private var isPreviewVisible = false

    private val availableCameraListener = object : ICameraStreamManager.AvailableCameraUpdatedListener {
        override fun onAvailableCameraUpdated(availableCameraList: MutableList<ComponentIndexType>) {
            mainHandler.post {
                updateCameraOptions(availableCameraList)
            }
        }

        override fun onCameraStreamEnableUpdate(cameraStreamEnableMap: MutableMap<ComponentIndexType, Boolean>) {
            // Not needed for this fragment
        }
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        return inflater.inflate(R.layout.frag_webrtc_stream, container, false)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initViews(view)
        setupListeners()
        setupCameraPreview()
        cameraStreamManager.addAvailableCameraUpdatedListener(availableCameraListener)
    }

    override fun onDestroyView() {
        super.onDestroyView()
        stopServer()
        removeCameraPreview()
        cameraStreamManager.removeAvailableCameraUpdatedListener(availableCameraListener)
        mainHandler.removeCallbacksAndMessages(null)
    }

    private fun initViews(view: View) {
        btnStartServer = view.findViewById(R.id.btn_start_server)
        btnStopServer = view.findViewById(R.id.btn_stop_server)
        rgCamera = view.findViewById(R.id.rg_camera)
        tvServerStatus = view.findViewById(R.id.tv_server_status)
        tvServerIp = view.findViewById(R.id.tv_server_ip)
        tvServerPort = view.findViewById(R.id.tv_server_port)
        tvClientCount = view.findViewById(R.id.tv_client_count)
        tvConnectionUrl = view.findViewById(R.id.tv_connection_url)
        tvStreamMetrics = view.findViewById(R.id.tv_stream_metrics)
        btnCopyUrl = view.findViewById(R.id.btn_copy_url)
        svCameraPreview = view.findViewById(R.id.sv_camera_preview)
        btnShowPreview = view.findViewById(R.id.btn_show_preview)
        btnHidePreview = view.findViewById(R.id.btn_hide_preview)
        tvErrorMessage = view.findViewById(R.id.tv_error_message)

        tvServerPort.text = DEFAULT_PORT.toString()
    updateMetrics(WebRTCStreamMetrics())
    }

    private fun setupListeners() {
        btnStartServer.setOnClickListener {
            startServer()
        }

        btnStopServer.setOnClickListener {
            stopServer()
        }

        rgCamera.setOnCheckedChangeListener { group, checkedId ->
            val view = group.findViewById<View>(checkedId)
            val tag = (view?.tag as? String)?.toIntOrNull() ?: 0
            selectedCameraIndex = ComponentIndexType.find(tag)
            
            // Update preview if visible
            if (isPreviewVisible) {
                updateCameraPreview()
            }
            
            // If server is running, we need to restart to apply camera change
            if (webRTCStreamer?.isRunning() == true) {
                ToastUtils.showToast(getString(R.string.webrtc_restart_required))
            }
        }

        btnCopyUrl.setOnClickListener {
            copyUrlToClipboard()
        }

        btnShowPreview.setOnClickListener {
            showPreview()
        }

        btnHidePreview.setOnClickListener {
            hidePreview()
        }
    }

    private fun setupCameraPreview() {
        svCameraPreview.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) {
                previewSurface = holder.surface
            }

            override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
                previewSurface = holder.surface
                previewWidth = width
                previewHeight = height
                if (isPreviewVisible) {
                    updateCameraPreview()
                }
            }

            override fun surfaceDestroyed(holder: SurfaceHolder) {
                removeCameraPreview()
                previewSurface = null
            }
        })
    }

    private fun updateCameraOptions(availableCameras: List<ComponentIndexType>) {
        // Update visibility of camera options based on available cameras
        val leftRadio = rgCamera.findViewById<View>(R.id.rb_camera_left)
        val rightRadio = rgCamera.findViewById<View>(R.id.rb_camera_right)
        val topRadio = rgCamera.findViewById<View>(R.id.rb_camera_top)
        val fpvRadio = rgCamera.findViewById<View>(R.id.rb_camera_fpv)

        leftRadio?.visibility = if (availableCameras.contains(ComponentIndexType.LEFT_OR_MAIN)) View.VISIBLE else View.GONE
        rightRadio?.visibility = if (availableCameras.contains(ComponentIndexType.RIGHT)) View.VISIBLE else View.GONE
        topRadio?.visibility = if (availableCameras.contains(ComponentIndexType.UP)) View.VISIBLE else View.GONE
        fpvRadio?.visibility = if (availableCameras.contains(ComponentIndexType.FPV)) View.VISIBLE else View.GONE
    }

    @SuppressLint("SetTextI18n")
    private fun startServer() {
        context?.applicationContext?.let { ctx ->
            webRTCStreamer = WebRTCStreamer(
                context = ctx,
                cameraIndex = selectedCameraIndex,
                signalingPort = DEFAULT_PORT,
                options = selectedOptions
            ).apply {
                listener = object : WebRTCStreamer.WebRTCStreamerListener {
                    override fun onServerStarted(ip: String, port: Int) {
                        mainHandler.post {
                            updateUIForServerStarted(ip, port)
                        }
                    }

                    override fun onServerStopped() {
                        mainHandler.post {
                            updateUIForServerStopped()
                        }
                    }

                    override fun onServerError(error: String) {
                        mainHandler.post {
                            showError(error)
                        }
                    }

                    override fun onClientConnected(clientId: String, totalClients: Int) {
                        mainHandler.post {
                            tvClientCount.text = totalClients.toString()
                            ToastUtils.showToast(getString(R.string.webrtc_client_connected))
                        }
                    }

                    override fun onClientDisconnected(clientId: String, totalClients: Int) {
                        mainHandler.post {
                            tvClientCount.text = totalClients.toString()
                            ToastUtils.showToast(getString(R.string.webrtc_client_disconnected))
                        }
                    }

                    override fun onMetrics(metrics: WebRTCStreamMetrics) {
                        mainHandler.post { updateMetrics(metrics) }
                    }
                }
                start()
            }
        }
    }

    private fun stopServer() {
        webRTCStreamer?.listener = null
        webRTCStreamer?.stop()
        webRTCStreamer = null
        updateUIForServerStopped()
    }

    @SuppressLint("SetTextI18n")
    private fun updateUIForServerStarted(ip: String, port: Int) {
        btnStartServer.isEnabled = false
        btnStopServer.isEnabled = true
        rgCamera.isEnabled = false
        
        // Disable camera radio buttons (camera change requires restart)
        for (i in 0 until rgCamera.childCount) {
            rgCamera.getChildAt(i).isEnabled = false
        }
        // Resolution radio buttons stay enabled (changes apply live)

        tvServerStatus.text = getString(R.string.webrtc_status_running)
        tvServerStatus.setTextColor(resources.getColor(android.R.color.holo_green_light, null))
        tvServerIp.text = ip
        tvServerPort.text = port.toString()
        tvConnectionUrl.text = "ws://$ip:$port"
        tvClientCount.text = "0"
        
        hideError()
        ToastUtils.showToast(getString(R.string.webrtc_server_started))
    }

    private fun updateUIForServerStopped() {
        btnStartServer.isEnabled = true
        btnStopServer.isEnabled = false
        rgCamera.isEnabled = true
        
        // Enable camera radio buttons
        for (i in 0 until rgCamera.childCount) {
            rgCamera.getChildAt(i).isEnabled = true
        }

        tvServerStatus.text = getString(R.string.webrtc_status_stopped)
        tvServerStatus.setTextColor(resources.getColor(android.R.color.holo_red_light, null))
        tvServerIp.text = getString(R.string.webrtc_ip_not_available)
        tvConnectionUrl.text = getString(R.string.webrtc_url_not_available)
        tvClientCount.text = "0"
        updateMetrics(WebRTCStreamMetrics())
    }

    private fun updateMetrics(metrics: WebRTCStreamMetrics) {
        tvStreamMetrics.text = metrics.compactLabel()
    }

    private fun showPreview() {
        isPreviewVisible = true
        btnShowPreview.visibility = View.GONE
        btnHidePreview.visibility = View.VISIBLE
        updateCameraPreview()
    }

    private fun hidePreview() {
        isPreviewVisible = false
        btnShowPreview.visibility = View.VISIBLE
        btnHidePreview.visibility = View.GONE
        removeCameraPreview()
    }

    private fun updateCameraPreview() {
        previewSurface?.let { surface ->
            if (previewWidth > 0 && previewHeight > 0) {
                cameraStreamManager.putCameraStreamSurface(
                    selectedCameraIndex,
                    surface,
                    previewWidth,
                    previewHeight,
                    ICameraStreamManager.ScaleType.CENTER_INSIDE
                )
            }
        }
    }

    private fun removeCameraPreview() {
        previewSurface?.let { surface ->
            cameraStreamManager.removeCameraStreamSurface(surface)
        }
    }

    private fun copyUrlToClipboard() {
        val url = tvConnectionUrl.text.toString()
        if (url.isNotEmpty() && url != getString(R.string.webrtc_url_not_available)) {
            val clipboard = requireContext().getSystemService(Context.CLIPBOARD_SERVICE) as ClipboardManager
            val clip = ClipData.newPlainText("WebRTC URL", url)
            clipboard.setPrimaryClip(clip)
            ToastUtils.showToast(getString(R.string.webrtc_url_copied))
        }
    }

    private fun showError(error: String) {
        tvErrorMessage.text = error
        tvErrorMessage.visibility = View.VISIBLE
    }

    private fun hideError() {
        tvErrorMessage.visibility = View.GONE
    }
}
