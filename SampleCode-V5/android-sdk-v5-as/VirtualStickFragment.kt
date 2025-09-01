/**
 * Initialize video feed components
 */
private fun initVideoFeed() {
    try {
        // Get the camera stream manager from MediaDataCenter
        cameraStreamManager = MediaDataCenter.getInstance().cameraStreamManager

        // Try to get reference to the TextureView if it exists in the layout
        // Note: This will be null if the layout doesn't have video_preview_surface
        try {
            videoPreviewSurface = binding?.root?.findViewById(R.id.video_preview_surface)
            if (videoPreviewSurface != null) {
                videoPreviewSurface?.surfaceTextureListener = this
                LogUtils.d("VirtualStickFragment", "Video feed TextureView found and initialized")
            } else {
                LogUtils.d("VirtualStickFragment", "No video_preview_surface found in layout - video feed disabled")
            }
        } catch (e: Exception) {
            LogUtils.w("VirtualStickFragment", "video_preview_surface not found in layout: ${e.message}")
            videoPreviewSurface = null
        }

        LogUtils.d("VirtualStickFragment", "Video feed initialization completed")

    } catch (e: Exception) {
        LogUtils.e("VirtualStickFragment", "Error initializing video feed: ${e.message}")
        ToastUtils.showToast("Failed to initialize video feed: ${e.message}")
    }
}

/**
 * Start video stream on the provided surface
 */
private fun startVideoStream(surface: SurfaceTexture) {
    try {
        if (videoPreviewSurface == null) {
            LogUtils.d("VirtualStickFragment", "Video feed disabled - no TextureView available")
            return
        }

        cameraStreamManager?.let { streamManager ->
            // The video feed functionality needs proper DJI V5 API implementation
            // For now, we log that the surface is available and ready for video feed
            LogUtils.d("VirtualStickFragment", "Video feed surface ready - waiting for proper DJI V5 camera stream setup")
            activity?.runOnUiThread {
                binding?.streamQualityInfoTv?.text = "Video Stream: Ready (awaiting camera connection)"
                ToastUtils.showToast("Video surface ready for drone camera feed")
            }
        } ?: run {
            LogUtils.e("VirtualStickFragment", "Camera stream manager is null")
            ToastUtils.showToast("Camera stream manager not available")
        }
    } catch (e: Exception) {
        LogUtils.e("VirtualStickFragment", "Error starting video stream: ${e.message}")
        ToastUtils.showToast("Failed to start video stream: ${e.message}")
    }
}

private val batteryKey: DJIKey<Int> = BatteryKey.KeyChargeRemainingInPercent.create()
private fun getBatteryLevel(): Int = batteryKey.get(-1)

// Add a listener to get battery updates
private fun setupBatteryListener() {
    LogUtils.i(TAG, "Setting up battery listener")

    // Check if the product is connected
    val isConnected = DJISDKManager.getInstance().isConnected
    LogUtils.i(TAG, "Product connected status: $isConnected")

    KeyManager.getInstance().listen(batteryKey,
        object : KeyListener<Int> {
            override fun onValueChange(oldValue: Int?, newValue: Int?) {
                LogUtils.i(TAG, "Battery value change: old=$oldValue, new=$newValue")
                activity?.runOnUiThread {
                    binding?.batteryLevelTextView?.text = "Battery Level: ${newValue ?: "N/A"}%"
                }
            }
        })

    // Try to get the initial value directly
    val initialValue = getBatteryLevel()
    LogUtils.i(TAG, "Initial battery value: $initialValue")
    if (initialValue >= 0) {
        activity?.runOnUiThread {
            binding?.batteryLevelTextView?.text = "Battery Level: $initialValue%"
        }
    } else {
        LogUtils.w(TAG, "Initial battery value not available")
    }

    // If not connected, update UI to show the reason for N/A
    if (!isConnected) {
        activity?.runOnUiThread {
            binding?.batteryLevelTextView?.text = "Battery Level: N/A (No Connection)%"
        }
    }
}

// Call this in your onViewCreated or similar lifecycle method
private fun initializeKeyListeners() {
    setupBatteryListener()
    // Other key listeners can be initialized here too
}
