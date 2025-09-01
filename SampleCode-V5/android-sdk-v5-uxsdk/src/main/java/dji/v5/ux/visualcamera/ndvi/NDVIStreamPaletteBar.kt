package dji.v5.ux.visualcamera.ndvi

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.utils.common.LogUtils
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.SchedulerProvider
import dji.v5.ux.core.base.widget.FrameLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.popover.PopoverHelper
import dji.v5.ux.databinding.UxsdkM3mStreamPaletteBarBinding
import dji.v5.ux.databinding.UxsdkPanelNdvlBinding

/**
 * NDVI 显示调色bar
 * 用于直接显示
 */
class NDVIStreamPaletteBar @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : FrameLayoutWidget<Any>(context, attrs), ICameraIndex, View.OnClickListener {

    private lateinit var binding: UxsdkM3mStreamPaletteBarBinding

    private val widgetModel by lazy {
        NDVIStreamPaletteBarPanelModel(DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance())
    }

    override fun getCameraIndex(): ComponentIndexType = widgetModel.getCameraIndex()

    override fun getLensType(): CameraLensType = widgetModel.getLensType()

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) {
        widgetModel.updateCameraSource(cameraIndex, lensType)
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkM3mStreamPaletteBarBinding.inflate(LayoutInflater.from(context),this,true)
        setOnClickListener(this)
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        if (!isInEditMode) {
            widgetModel.setup()
        }
    }

    override fun onDetachedFromWindow() {
        if (!isInEditMode) {
            widgetModel.cleanup()
        }
        super.onDetachedFromWindow()
    }

    override fun reactToModelChanges() {
        addReaction(widgetModel.multiSpectralFusionDisplayRangeProcessor.toFlowable()
            .observeOn(SchedulerProvider.ui())
            .subscribe {
                binding.leftTv.text = (it.displayMin / 10.0f).toString()
                binding.rightTv.text = (it.displayMax / 10.0f).toString()
            }
        )
    }

    override fun onClick(v: View?) {
        val view = NDVIStreamPopoverViewWidget(context)
        view.updateCameraSource(getCameraIndex(), getLensType())
        view.selectIndex = 1
        PopoverHelper.showPopover(binding.streamPaletteRootView, view)
    }
}