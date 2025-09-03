package dji.sampleV5.moduleaircraft.controller
import kotlin.math.max
import kotlin.math.min

/**
 * This class implements the PID functionality with anti-windup
 */
class PID(
        private val kp: Double,
        private val ki: Double,
        private val kd: Double,
        private val dt: Double,
        private val outputLimits: Pair<Double?, Double?>
) {
    private var integral = 0.0
    private var previousError = 0.0

    fun update(error: Double): Double {
        // Proportional term
        val p = kp * error

        // Derivative term
        val derivative = if (dt != 0.0) (error - previousError) / dt else 0.0
        val d = kd * derivative
        previousError = error

        // Integral term with anti-windup
        integral += error * dt

        // Calculate output before applying integral term
        var output = p + d

        // Apply output limits to check for saturation
        val (minOutput, maxOutput) = outputLimits
        val outputUnclamped = output + ki * integral

        // Anti-windup: Only update integral if output is not saturated
        val i = if ((minOutput == null || outputUnclamped > minOutput) &&
                (maxOutput == null || outputUnclamped < maxOutput)) {
            ki * integral
        } else {
            0.0
        }

        // PID output before limits
        output += i

        // Apply output limits
        if (minOutput != null) output = max(minOutput, output)
        if (maxOutput != null) output = min(maxOutput, output)

        return output
    }
}