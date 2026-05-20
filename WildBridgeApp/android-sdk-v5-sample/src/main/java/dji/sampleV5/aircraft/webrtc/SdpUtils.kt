package dji.sampleV5.aircraft.webrtc

import android.util.Log

/**
 * SDP manipulation utilities for forcing H264 codec and tuning keyframe interval.
 *
 * The DefaultVideoEncoderFactory always registers a software VP8 encoder even when
 * the Intel VP8 hardware encoder is disabled. During SDP negotiation the remote peer
 * (e.g. MediaMTX) may prefer VP8, causing the stream to be encoded as VP8 instead of
 * H264. These helpers strip non-H264 video codecs from the SDP so only H264 can be
 * negotiated.
 */
object SdpUtils {
    private const val TAG = "SdpUtils"

    /**
     * Remove all video codecs except H264 (and their associated RTX/RED/ULPFEC)
     * from the SDP, forcing the peer connection to negotiate H264.
     *
     * If no H264 codec is found the original SDP is returned unchanged.
     */
    fun forceH264Only(sdp: String): String {
        val lineEnding = if ("\r\n" in sdp) "\r\n" else "\n"
        val lines = sdp.split(lineEnding)

        // --- Phase 1: locate the video m-line and collect codec metadata ---
        var videoLineIdx = -1
        var videoEndIdx = lines.size
        val allVideoPts = mutableListOf<Int>()
        val codecForPt = mutableMapOf<Int, String>()
        val rtxApt = mutableMapOf<Int, Int>() // rtx_pt -> associated_pt

        for ((i, line) in lines.withIndex()) {
            if (line.startsWith("m=video")) {
                videoLineIdx = i
                line.split(" ").drop(3).forEach { tok ->
                    tok.toIntOrNull()?.let { allVideoPts.add(it) }
                }
            } else if (videoLineIdx >= 0 && i > videoLineIdx && line.startsWith("m=")) {
                videoEndIdx = i
                break
            }

            if (videoLineIdx >= 0 && i > videoLineIdx) {
                Regex("""^a=rtpmap:(\d+)\s+(\S+)/""").find(line)?.let { m ->
                    codecForPt[m.groupValues[1].toInt()] = m.groupValues[2]
                }
                Regex("""^a=fmtp:(\d+)\s+.*\bapt=(\d+)""").find(line)?.let { m ->
                    rtxApt[m.groupValues[1].toInt()] = m.groupValues[2].toInt()
                }
            }
        }

        if (videoLineIdx < 0) return sdp

        val h264Pts = codecForPt.filter { it.value.equals("H264", ignoreCase = true) }.keys
        if (h264Pts.isEmpty()) {
            Log.w(TAG, "No H264 codec found in SDP — leaving unchanged")
            return sdp
        }

        val h264Rtx = rtxApt.filter { it.value in h264Pts }.keys
        val redPts = codecForPt.filter { it.value.equals("red", ignoreCase = true) }.keys
        val ulpfecPts = codecForPt.filter { it.value.equals("ulpfec", ignoreCase = true) }.keys
        val allowed = h264Pts + h264Rtx + redPts + ulpfecPts

        // --- Phase 2: rebuild the SDP keeping only allowed payload types ---
        val result = mutableListOf<String>()

        for ((i, line) in lines.withIndex()) {
            if (i == videoLineIdx) {
                // Rewrite m=video line with only the allowed PTs
                val parts = line.split(" ")
                val prefix = parts.take(3).joinToString(" ")
                val kept = allVideoPts.filter { it in allowed }
                result.add("$prefix ${kept.joinToString(" ")}")
                continue
            }

            if (i in (videoLineIdx + 1) until videoEndIdx) {
                val ptMatch = Regex("""^a=(?:rtpmap|fmtp|rtcp-fb):(\d+)\b""").find(line)
                if (ptMatch != null && ptMatch.groupValues[1].toInt() !in allowed) {
                    continue // drop lines for disallowed codecs
                }
            }

            result.add(line)
        }

        Log.d(TAG, "Forced H264-only: kept PTs ${allowed.joinToString(",")}")
        return result.joinToString(lineEnding)
    }

    /**
     * Inject `x-google-max-keyframe-interval` into every H264 fmtp line.
     * This tells the libwebrtc encoder to insert a keyframe at most every
     * [intervalMs] milliseconds, improving recovery from packet loss.
     *
     * @param intervalMs max interval between keyframes in milliseconds (e.g. 2000)
     */
    fun setKeyframeInterval(sdp: String, intervalMs: Int): String {
        if (intervalMs <= 0) return sdp

        val lineEnding = if ("\r\n" in sdp) "\r\n" else "\n"
        val lines = sdp.split(lineEnding)

        // Collect H264 payload types
        val h264Pts = mutableSetOf<Int>()
        for (line in lines) {
            Regex("""^a=rtpmap:(\d+)\s+H264/""").find(line)?.let {
                h264Pts.add(it.groupValues[1].toInt())
            }
        }
        if (h264Pts.isEmpty()) return sdp

        val result = lines.map { line ->
            val fmtpMatch = Regex("""^a=fmtp:(\d+)\s+""").find(line)
            if (fmtpMatch != null && fmtpMatch.groupValues[1].toInt() in h264Pts
                && "x-google-max-keyframe-interval" !in line
            ) {
                "$line;x-google-max-keyframe-interval=$intervalMs"
            } else {
                line
            }
        }

        return result.joinToString(lineEnding)
    }

    /**
     * Apply both H264 enforcement and keyframe interval to an SDP string.
     */
    fun mungeForH264(sdp: String, keyframeIntervalMs: Int = 2000): String =
        setKeyframeInterval(forceH264Only(sdp), keyframeIntervalMs)
}
