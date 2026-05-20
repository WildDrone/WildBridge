package dji.v5.ux.detection

import org.json.JSONArray
import org.json.JSONObject

/**
 * Represents a target detected by the drone's onboard AutoSensing AI.
 * Bounding box coordinates are normalised [0.0–1.0] relative to frame dimensions.
 */
data class DetectedTarget(
    val index: Int,
    val type: String,         // e.g. "PERSON", "VEHICLE", "BOAT", "ANIMAL", "UNKNOWN"
    val left: Double,
    val top: Double,
    val right: Double,
    val bottom: Double
) {
    fun toJson(): JSONObject = JSONObject().apply {
        put("index", index)
        put("type", type)
        put("rect", JSONArray().apply {
            put(left); put(top); put(right); put(bottom)
        })
    }

    companion object {
        fun listToJsonArray(targets: List<DetectedTarget>): JSONArray {
            val arr = JSONArray()
            targets.forEach { arr.put(it.toJson()) }
            return arr
        }

        fun fromJsonArray(arr: JSONArray): List<DetectedTarget> {
            val list = mutableListOf<DetectedTarget>()
            for (i in 0 until arr.length()) {
                val obj = arr.getJSONObject(i)
                val rect = obj.getJSONArray("rect")
                list.add(
                    DetectedTarget(
                        index = obj.getInt("index"),
                        type = obj.getString("type"),
                        left = rect.getDouble(0),
                        top = rect.getDouble(1),
                        right = rect.getDouble(2),
                        bottom = rect.getDouble(3)
                    )
                )
            }
            return list
        }
    }
}
