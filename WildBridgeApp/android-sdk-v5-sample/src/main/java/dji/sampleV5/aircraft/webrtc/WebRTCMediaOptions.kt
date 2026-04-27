package dji.sampleV5.aircraft.webrtc

/**
 * Configuration options for WebRTC media streaming.
 */
data class WebRTCMediaOptions(
    val mediaStreamId: String = "DJI_DRONE_STREAM",
    val videoTrackId: String = "DJI_VIDEO_TRACK",
    val videoResolutionWidth: Int = 1920,
    val videoResolutionHeight: Int = 1080,
    val fps: Int = 30,
    val videoBitrate: Int = 8_000_000,  // 8 Mbps (suitable for 1080p H264)
    val videoCodec: String = "H264"     // H264 High Profile for best quality
) {
    companion object {
        /** 1920x1080 @ 8 Mbps — best quality for detection */
        fun fullHD() = WebRTCMediaOptions(
            videoResolutionWidth = 1920,
            videoResolutionHeight = 1080,
            videoBitrate = 8_000_000,
            videoCodec = "H264"
        )

        /** 1280x720 @ 4 Mbps — lighter on bandwidth */
        fun hd() = WebRTCMediaOptions(
            videoResolutionWidth = 1280,
            videoResolutionHeight = 720,
            videoBitrate = 4_000_000,
            videoCodec = "H264"
        )

        /** 640x480 @ 1.5 Mbps — low bandwidth fallback */
        fun sd() = WebRTCMediaOptions(
            videoResolutionWidth = 640,
            videoResolutionHeight = 480,
            videoBitrate = 1_500_000,
            videoCodec = "H264"
        )
    }
}
