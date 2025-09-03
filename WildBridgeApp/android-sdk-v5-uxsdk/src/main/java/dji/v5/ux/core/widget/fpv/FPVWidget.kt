/*
 * Copyright (c) 2018-2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
package dji.v5.ux.core.widget.fpv

import android.annotation.SuppressLint
import android.content.Context
import android.content.res.ColorStateList
import android.graphics.drawable.Drawable
import android.util.AttributeSet
import android.view.Surface
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.widget.TextView
import androidx.annotation.ColorInt
import androidx.annotation.Dimension
import androidx.annotation.FloatRange
import androidx.annotation.StyleRes
import androidx.constraintlayout.widget.Guideline
import androidx.core.content.res.use
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.utils.common.DisplayUtil
import dji.v5.utils.common.LogPath
import dji.v5.utils.common.LogUtils
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.SchedulerProvider
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.extension.*
import dji.v5.ux.core.module.FlatCameraModule
import dji.v5.ux.core.ui.CenterPointView
import dji.v5.ux.core.ui.GridLineView
import dji.v5.ux.core.util.UxErrorHandle
import dji.v5.ux.core.widget.fpv.FPVWidget.ModelState
import io.reactivex.rxjava3.core.Flowable

private const val TAG = "FPVWidget"
private const val ORIGINAL_SCALE = 1f
private const val LANDSCAPE_ROTATION_ANGLE = 0

/**
 * This widget shows the video feed from the camera.
 */
open class FPVWidget @JvmOverloads constructor(
    context: Context, attrs: AttributeSet? = null, defStyleAttr: Int = 0
) : ConstraintLayoutWidget<ModelState>(context, attrs, defStyleAttr) {
    private var viewWidth = 0
    private var viewHeight = 0
    private var rotationAngle = 0
    private var surface: Surface? = null
    private var width = -1
    private var height = -1
    private val fpvSurfaceView: SurfaceView = findViewById(R.id.surface_view_fpv)
    private val cameraNameTextView: TextView = findViewById(R.id.textview_camera_name)
    private val cameraSideTextView: TextView = findViewById(R.id.textview_camera_side)
    private val verticalOffset: Guideline = findViewById(R.id.vertical_offset)
    private val horizontalOffset: Guideline = findViewById(R.id.horizontal_offset)
    private var fpvStateChangeResourceId: Int = INVALID_RESOURCE

    private val cameraSurfaceCallback = object : SurfaceHolder.Callback {
        override fun surfaceCreated(holder: SurfaceHolder) {
            surface = holder.surface
            LogUtils.i(LogPath.SAMPLE, "surfaceCreated: ${widgetModel.getCameraIndex()}")
        }

        override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
            this@FPVWidget.width = width
            this@FPVWidget.height = height
            LogUtils.i(LogPath.SAMPLE, "surfaceChanged: ${widgetModel.getCameraIndex()}", "width:$width", ",height:$height")
            updateCameraStream()
        }

        override fun surfaceDestroyed(holder: SurfaceHolder) {
            width = 0
            height = 0
            LogUtils.i(LogPath.SAMPLE, "surfaceDestroyed: ${widgetModel.getCameraIndex()}")
            removeSurfaceBinding()
        }
    }

    val widgetModel: FPVWidgetModel = FPVWidgetModel(
        DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance(), FlatCameraModule()
    )

    /**
     * Whether the video feed source's camera name is visible on the video feed.
     */
    var isCameraSourceNameVisible = true
        set(value) {
            field = value
            checkAndUpdateCameraName()
        }

    /**
     * Whether the video feed source's camera side is visible on the video feed.
     * Only shown on aircraft that support multiple gimbals.
     */
    var isCameraSourceSideVisible = true
        set(value) {
            field = value
            checkAndUpdateCameraSide()
        }

    /**
     * Whether the grid lines are enabled.
     */
    var isGridLinesEnabled = true
        set(isGridLinesEnabled) {
            field = isGridLinesEnabled
            updateGridLineVisibility()
        }

    /**
     * Whether the center point is enabled.
     */
    var isCenterPointEnabled = true
        set(isCenterPointEnabled) {
            field = isCenterPointEnabled
            centerPointView.visibility = if (isCenterPointEnabled) View.VISIBLE else View.GONE
        }

    /**
     * The text color state list of the camera name text view
     */
    var cameraNameTextColors: ColorStateList?
        get() = cameraNameTextView.textColors
        set(colorStateList) {
            cameraNameTextView.setTextColor(colorStateList)
        }

    /**
     * The text color of the camera name text view
     */
    @get:ColorInt
    @setparam:ColorInt
    var cameraNameTextColor: Int
        get() = cameraNameTextView.currentTextColor
        set(color) {
            cameraNameTextView.setTextColor(color)
        }

    /**
     * The text size of the camera name text view
     */
    @get:Dimension
    @setparam:Dimension
    var cameraNameTextSize: Float
        get() = cameraNameTextView.textSize
        set(textSize) {
            cameraNameTextView.textSize = textSize
        }

    /**
     * The background for the camera name text view
     */
    var cameraNameTextBackground: Drawable?
        get() = cameraNameTextView.background
        set(drawable) {
            cameraNameTextView.background = drawable
        }

    /**
     * The text color state list of the camera name text view
     */
    var cameraSideTextColors: ColorStateList?
        get() = cameraSideTextView.textColors
        set(colorStateList) {
            cameraSideTextView.setTextColor(colorStateList)
        }

    /**
     * The text color of the camera side text view
     */
    @get:ColorInt
    @setparam:ColorInt
    var cameraSideTextColor: Int
        get() = cameraSideTextView.currentTextColor
        set(color) {
            cameraSideTextView.setTextColor(color)
        }

    /**
     * The text size of the camera side text view
     */
    @get:Dimension
    @setparam:Dimension
    var cameraSideTextSize: Float
        get() = cameraSideTextView.textSize
        set(textSize) {
            cameraSideTextView.textSize = textSize
        }

    /**
     * The background for the camera side text view
     */
    var cameraSideTextBackground: Drawable?
        get() = cameraSideTextView.background
        set(drawable) {
            cameraSideTextView.background = drawable
        }

    /**
     * The vertical alignment of the camera name and side text views
     */
    var cameraDetailsVerticalAlignment: Float
        @FloatRange(from = 0.0, to = 1.0) get() {
            val layoutParams: LayoutParams = verticalOffset.layoutParams as LayoutParams
            return layoutParams.guidePercent
        }
        set(@FloatRange(from = 0.0, to = 1.0) percent) {
            val layoutParams: LayoutParams = verticalOffset.layoutParams as LayoutParams
            layoutParams.guidePercent = percent
            verticalOffset.layoutParams = layoutParams
        }

    /**
     * The horizontal alignment of the camera name and side text views
     */
    var cameraDetailsHorizontalAlignment: Float
        @FloatRange(from = 0.0, to = 1.0) get() {
            val layoutParams: LayoutParams = horizontalOffset.layoutParams as LayoutParams
            return layoutParams.guidePercent
        }
        set(@FloatRange(from = 0.0, to = 1.0) percent) {
            val layoutParams: LayoutParams = horizontalOffset.layoutParams as LayoutParams
            layoutParams.guidePercent = percent
            horizontalOffset.layoutParams = layoutParams
        }

    /**
     * The [GridLineView] shown in this widget
     */
    val gridLineView: GridLineView = findViewById(R.id.view_grid_line)

    /**
     * The [CenterPointView] shown in this widget
     */
    val centerPointView: CenterPointView = findViewById(R.id.view_center_point)

    //endregion

    //region Constructor
    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        inflate(context, R.layout.uxsdk_widget_fpv, this)
    }

    init {
        if (!isInEditMode) {
            rotationAngle = LANDSCAPE_ROTATION_ANGLE
            fpvSurfaceView.holder.addCallback(cameraSurfaceCallback)
        }
        attrs?.let { initAttributes(context, it) }
    }
    //endregion

    //region LifeCycle
    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        if (!isInEditMode) {
            widgetModel.setup()
        }
        initializeListeners()
    }

    private fun initializeListeners() {
        //后面补上
    }

    override fun setVisibility(visibility: Int) {
        super.setVisibility(visibility)
        fpvSurfaceView.visibility = visibility
    }

    override fun onDetachedFromWindow() {
        destroyListeners()
        if (!isInEditMode) {
            widgetModel.cleanup()
        }
        super.onDetachedFromWindow()
    }

    override fun reactToModelChanges() {
        addReaction(widgetModel.displayMsgProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe { cameraName: String -> updateCameraName(cameraName) })
        addReaction(widgetModel.cameraSideProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe { cameraSide: String -> updateCameraSide(cameraSide) })
        addReaction(widgetModel.hasVideoViewChanged.observeOn(SchedulerProvider.ui()).subscribe { delayCalculator() })
    }

    override fun onLayout(changed: Boolean, l: Int, t: Int, r: Int, b: Int) {
        super.onLayout(changed, l, t, r, b)
        if (!isInEditMode) {
            setViewDimensions()
            delayCalculator()
        }
    }

    private fun destroyListeners() {
        //后面补上
    }

    //endregion
    //region Customization
    override fun getIdealDimensionRatioString(): String {
        return getString(R.string.uxsdk_widget_fpv_ratio)
    }

    fun updateVideoSource(source: ComponentIndexType) {
        LogUtils.i(LogPath.SAMPLE, "updateVideoSource", source, this)
        widgetModel.updateCameraSource(source, CameraLensType.UNKNOWN)
        updateCameraStream()
        if (source == ComponentIndexType.VISION_ASSIST) {
            widgetModel.enableVisionAssist()
        }
        fpvSurfaceView.invalidate()
    }

    fun setOnFPVStreamSourceListener(listener: FPVStreamSourceListener) {
        widgetModel.streamSourceListener = listener
    }

    fun setSurfaceViewZOrderOnTop(onTop: Boolean) {
        fpvSurfaceView.setZOrderOnTop(onTop)
    }

    fun setSurfaceViewZOrderMediaOverlay(isMediaOverlay: Boolean) {
        fpvSurfaceView.setZOrderMediaOverlay(isMediaOverlay)
    }

    //endregion
    //region Helpers
    private fun setViewDimensions() {
        viewWidth = measuredWidth
        viewHeight = measuredHeight
    }

    private fun delayCalculator() {
        //后面补充
    }

    private fun updateCameraName(cameraName: String) {
        cameraNameTextView.text = cameraName
        if (cameraName.isNotEmpty() && isCameraSourceNameVisible) {
            cameraNameTextView.visibility = View.VISIBLE
        } else {
            cameraNameTextView.visibility = View.INVISIBLE
        }
    }

    private fun updateCameraSide(cameraSide: String) {
        cameraSideTextView.text = cameraSide
        if (cameraSide.isNotEmpty() && isCameraSourceSideVisible) {
            cameraSideTextView.visibility = View.VISIBLE
        } else {
            cameraSideTextView.visibility = View.INVISIBLE
        }
    }

    private fun checkAndUpdateCameraName() {
        if (!isInEditMode) {
            addDisposable(
                widgetModel.displayMsgProcessor.toFlowable().firstOrError().observeOn(SchedulerProvider.ui()).subscribe(
                    { cameraName: String -> updateCameraName(cameraName) }, UxErrorHandle.logErrorConsumer(TAG, "updateCameraName")
                )
            )
        }
    }

    private fun checkAndUpdateCameraSide() {
        if (!isInEditMode) {
            addDisposable(
                widgetModel.cameraSideProcessor.toFlowable().firstOrError().observeOn(SchedulerProvider.ui()).subscribe(
                    { cameraSide: String -> updateCameraSide(cameraSide) }, UxErrorHandle.logErrorConsumer(TAG, "updateCameraSide")
                )
            )
        }
    }

    private fun updateGridLineVisibility() {
        gridLineView.visibility = if (isGridLinesEnabled && widgetModel.getCameraIndex() == ComponentIndexType.FPV) View.VISIBLE else View.GONE
    }
    //endregion

    //region Customization helpers
    /**
     * Set text appearance of the camera name text view
     *
     * @param textAppearance Style resource for text appearance
     */
    fun setCameraNameTextAppearance(@StyleRes textAppearance: Int) {
        cameraNameTextView.setTextAppearance(context, textAppearance)
    }

    /**
     * Set text appearance of the camera side text view
     *
     * @param textAppearance Style resource for text appearance
     */
    fun setCameraSideTextAppearance(@StyleRes textAppearance: Int) {
        cameraSideTextView.setTextAppearance(context, textAppearance)
    }

    @SuppressLint("Recycle")
    private fun initAttributes(context: Context, attrs: AttributeSet) {
        context.obtainStyledAttributes(attrs, R.styleable.FPVWidget).use { typedArray ->
            if (!isInEditMode) {
                typedArray.getBooleanAndUse(R.styleable.FPVWidget_uxsdk_gridLinesEnabled, true) {
                    isGridLinesEnabled = it
                }
                typedArray.getBooleanAndUse(R.styleable.FPVWidget_uxsdk_centerPointEnabled, true) {
                    isCenterPointEnabled = it
                }
            }
            typedArray.getBooleanAndUse(R.styleable.FPVWidget_uxsdk_sourceCameraNameVisibility, true) {
                isCameraSourceNameVisible = it
            }
            typedArray.getBooleanAndUse(R.styleable.FPVWidget_uxsdk_sourceCameraSideVisibility, true) {
                isCameraSourceSideVisible = it
            }
            typedArray.getResourceIdAndUse(R.styleable.FPVWidget_uxsdk_cameraNameTextAppearance) {
                setCameraNameTextAppearance(it)
            }
            typedArray.getDimensionAndUse(R.styleable.FPVWidget_uxsdk_cameraNameTextSize) {
                cameraNameTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getColorAndUse(R.styleable.FPVWidget_uxsdk_cameraNameTextColor) {
                cameraNameTextColor = it
            }
            typedArray.getDrawableAndUse(R.styleable.FPVWidget_uxsdk_cameraNameBackgroundDrawable) {
                cameraNameTextBackground = it
            }
            typedArray.getResourceIdAndUse(R.styleable.FPVWidget_uxsdk_cameraSideTextAppearance) {
                setCameraSideTextAppearance(it)
            }
            typedArray.getDimensionAndUse(R.styleable.FPVWidget_uxsdk_cameraSideTextSize) {
                cameraSideTextSize = DisplayUtil.pxToSp(context, it)
            }
            typedArray.getColorAndUse(R.styleable.FPVWidget_uxsdk_cameraSideTextColor) {
                cameraSideTextColor = it
            }
            typedArray.getDrawableAndUse(R.styleable.FPVWidget_uxsdk_cameraSideBackgroundDrawable) {
                cameraSideTextBackground = it
            }
            typedArray.getFloatAndUse(R.styleable.FPVWidget_uxsdk_cameraDetailsVerticalAlignment) {
                cameraDetailsVerticalAlignment = it
            }
            typedArray.getFloatAndUse(R.styleable.FPVWidget_uxsdk_cameraDetailsHorizontalAlignment) {
                cameraDetailsHorizontalAlignment = it
            }
            typedArray.getIntegerAndUse(R.styleable.FPVWidget_uxsdk_gridLineType) {
                gridLineView.type = GridLineView.GridLineType.find(it)
            }
            typedArray.getColorAndUse(R.styleable.FPVWidget_uxsdk_gridLineColor) {
                gridLineView.lineColor = it
            }
            typedArray.getFloatAndUse(R.styleable.FPVWidget_uxsdk_gridLineWidth) {
                gridLineView.lineWidth = it
            }
            typedArray.getIntegerAndUse(R.styleable.FPVWidget_uxsdk_gridLineNumber) {
                gridLineView.numberOfLines = it
            }
            typedArray.getIntegerAndUse(R.styleable.FPVWidget_uxsdk_centerPointType) {
                centerPointView.type = CenterPointView.CenterPointType.find(it)
            }
            typedArray.getColorAndUse(R.styleable.FPVWidget_uxsdk_centerPointColor) {
                centerPointView.color = it
            }
            typedArray.getResourceIdAndUse(R.styleable.FPVWidget_uxsdk_onStateChange) {
                fpvStateChangeResourceId = it
            }
        }
    }

    private fun updateCameraStream() {
        removeSurfaceBinding()
        surface?.let {
            widgetModel.putCameraStreamSurface(
                it,
                width,
                height,
                ICameraStreamManager.ScaleType.CENTER_INSIDE
            )
        }
    }

    private fun removeSurfaceBinding() {
        if (width <= 0 || height <= 0 || surface == null) {
            if (surface != null) {
                widgetModel.removeCameraStreamSurface(surface!!)
            }
        }
    }

    /**
     * Get the [ModelState] updates
     */
    @SuppressWarnings
    override fun getWidgetStateUpdate(): Flowable<ModelState> {
        return super.getWidgetStateUpdate()
    }

    /**
     * Class defines the widget state updates
     */
    sealed class ModelState
}