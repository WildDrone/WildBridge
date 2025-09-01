package dji.v5.ux.visualcamera.ndvi

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.SchedulerProvider.ui
import dji.v5.ux.core.base.widget.FrameLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.extension.getColor
import dji.v5.ux.core.extension.getString
import dji.v5.ux.databinding.UxsdkCameraStatusActionItemContentBinding
import dji.v5.ux.databinding.UxsdkPanelCommonCameraBinding
import dji.v5.ux.databinding.UxsdkPanelNdvlBinding


/**
 * 仅用于CameraLensType.CAMERA_LENS_MS_NDVI镜头下
 */
open class SpectralDisplayModeWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : FrameLayoutWidget<Any>(context, attrs, defStyleAttr), ICameraIndex, View.OnClickListener {

    private lateinit var binding: UxsdkCameraStatusActionItemContentBinding

    private val widgetModel by lazy {
        SpectralDisplayModeWidgetModel(DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance())
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkCameraStatusActionItemContentBinding.inflate(LayoutInflater.from(context), this, true)
        setOnClickListener(this)
    }

    override fun onClick(v: View?) {
        widgetModel.changeDisplayMode()
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        if (!isInEditMode) {
            widgetModel.setup()
        }
        binding.tvContent.text = getString(R.string.uxsdk_stream_switcher_sbs)
    }

    override fun onDetachedFromWindow() {
        if (!isInEditMode) {
            widgetModel.cleanup()
        }
        super.onDetachedFromWindow()
    }

    override fun reactToModelChanges() {
        addReaction(widgetModel.isSBSOnProcessor.toFlowable()
            .observeOn(ui())
            .subscribe {
                if (it) {
                    binding.tvContent.setTextColor(getColor(R.color.uxsdk_yellow_in_light))
                } else {
                    binding.tvContent.setTextColor(getColor(R.color.uxsdk_white_33_percent))
                }
            }
        )
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