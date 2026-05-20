package dji.v5.ux.detection

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.RectF
import android.util.AttributeSet
import android.view.View

/**
 * Transparent overlay drawn on top of the FPV widget to render
 * AutoSensing detection bounding boxes in real-time.
 */
class DetectionOverlayView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    @Volatile
    private var targets: List<DetectedTarget> = emptyList()

    private val boxPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
        color = Color.parseColor("#00E676") // bright green
    }

    private val labelBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.parseColor("#AA000000") // semi-transparent black
    }

    private val labelTextPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.WHITE
        textSize = 28f
        isFakeBoldText = true
    }

    /**
     * Update the list of detected targets (called from any thread).
     */
    fun setTargets(newTargets: List<DetectedTarget>) {
        targets = newTargets
        postInvalidate()
    }

    fun clearTargets() {
        targets = emptyList()
        postInvalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        val w = width.toFloat()
        val h = height.toFloat()
        if (w == 0f || h == 0f) return

        val snapshot = targets
        for (target in snapshot) {
            val rect = RectF(
                (target.left * w).toFloat(),
                (target.top * h).toFloat(),
                (target.right * w).toFloat(),
                (target.bottom * h).toFloat()
            )
            canvas.drawRect(rect, boxPaint)

            // Label
            val label = "#${target.index} ${target.type}"
            val textWidth = labelTextPaint.measureText(label)
            val textHeight = labelTextPaint.textSize
            val labelLeft = rect.left
            val labelTop = (rect.top - textHeight - 6f).coerceAtLeast(0f)
            canvas.drawRect(labelLeft, labelTop, labelLeft + textWidth + 12f, labelTop + textHeight + 6f, labelBgPaint)
            canvas.drawText(label, labelLeft + 6f, labelTop + textHeight, labelTextPaint)
        }
    }
}
