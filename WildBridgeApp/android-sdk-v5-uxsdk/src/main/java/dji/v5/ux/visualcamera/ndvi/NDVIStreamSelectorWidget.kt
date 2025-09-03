package dji.v5.ux.visualcamera.ndvi

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.camera.CameraVideoStreamSourceType
import dji.sdk.keyvalue.value.camera.MultiSpectralFusionType
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.SchedulerProvider
import dji.v5.ux.core.base.widget.FrameLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.popover.PopoverHelper
import dji.v5.ux.databinding.UxsdkCameraStatusActionItemContentBinding
import dji.v5.ux.databinding.UxsdkPanelNdvlBinding

open class NDVIStreamSelectorWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : FrameLayoutWidget<Any>(context, attrs, defStyleAttr), ICameraIndex, View.OnClickListener {

    private lateinit var binding: UxsdkCameraStatusActionItemContentBinding

    private val widgetModel by lazy {
        NDVIStreamSelectorWidgetModel(DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance())
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        if (!isInEditMode) {
            widgetModel.setup()
        }
        binding.tvContent.text = "NDVI"
    }

    override fun onDetachedFromWindow() {
        if (!isInEditMode) {
            widgetModel.cleanup()
        }
        super.onDetachedFromWindow()
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkCameraStatusActionItemContentBinding.inflate(LayoutInflater.from(context),this,true)
        setOnClickListener(this)
    }

    override fun reactToModelChanges() {
        addReaction(widgetModel.cameraVideoStreamSourceProcessor.toFlowable()
            .observeOn(SchedulerProvider.ui())
            .subscribe {
                updateContent()
            }
        )
        addReaction(widgetModel.cameraMultiSpectralFusionTypeProcessor.toFlowable()
            .observeOn(SchedulerProvider.ui())
            .subscribe {
                updateContent()
            }
        )
    }

    private fun updateContent() {
        binding.tvContent.text = when (widgetModel.cameraVideoStreamSourceProcessor.value) {
            CameraVideoStreamSourceType.MS_G_CAMERA -> "G"
            CameraVideoStreamSourceType.MS_R_CAMERA -> "R"
            CameraVideoStreamSourceType.MS_RE_CAMERA -> "RE"
            CameraVideoStreamSourceType.MS_NIR_CAMERA -> "NIR"
            CameraVideoStreamSourceType.NDVI_CAMERA -> {
                when (widgetModel.cameraMultiSpectralFusionTypeProcessor.value) {
                    MultiSpectralFusionType.GNDVI -> {
                        "GNDVI"
                    }
                    MultiSpectralFusionType.NDRE -> {
                        "NDRE"
                    }
                    else -> {
                        "NDVI"
                    }
                }
            }
            else -> ""
        }
    }

    override fun onClick(v: View?) {
        openSettingPanel()
    }

    private fun openSettingPanel() {
        val view = NDVIStreamPopoverViewWidget(context)
        view.updateCameraSource(getCameraIndex(),getLensType())
        view.selectIndex = 0
        PopoverHelper.showPopover(binding.streamSelectorRootView, view)
    }

    override fun getCameraIndex(): ComponentIndexType {
        return widgetModel.getCameraIndex()
    }

    override fun getLensType(): CameraLensType {
        return widgetModel.getLensType()
    }

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) {
        widgetModel.updateCameraSource(cameraIndex, lensType)
    }
}