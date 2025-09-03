package dji.v5.ux.cameracore.widget.cameracontrols.exposuresettings

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.camera.CameraExposureMode
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.SchedulerProvider
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.util.UxErrorHandle
import dji.v5.ux.databinding.UxsdkPanelNdvlBinding
import dji.v5.ux.databinding.UxsdkWidgetExposureModeSettingBinding
import io.reactivex.rxjava3.functions.Action

/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/10/19
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */
open class ExposureModeSettingWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : ConstraintLayoutWidget<ExposureModeSettingWidget.ModelState>(context, attrs, defStyleAttr),
    View.OnClickListener, ICameraIndex {

    private lateinit var binding: UxsdkWidgetExposureModeSettingBinding

    private val widgetModel by lazy {
        ExposureModeSettingModel(DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance())
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkWidgetExposureModeSettingBinding.inflate(LayoutInflater.from(context), this)
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        if (!isInEditMode) {
            widgetModel.setup()
        }
        binding.layoutCameraModeA.setOnClickListener(this)
        binding.layoutCameraModeS.setOnClickListener(this)
        binding.layoutCameraModeM.setOnClickListener(this)
        binding.layoutCameraModeP.setOnClickListener(this)
        binding.layoutCameraModeP.isSelected = true
    }

    override fun onDetachedFromWindow() {
        if (!isInEditMode) {
            widgetModel.cleanup()
        }
        super.onDetachedFromWindow()
    }

    override fun getCameraIndex() = widgetModel.getCameraIndex()

    override fun getLensType() = widgetModel.getLensType()

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) = widgetModel.updateCameraSource(cameraIndex, lensType)

    override fun reactToModelChanges() {
        addReaction(widgetModel.exposureModeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            updateExposureMode(it)
        })
        addReaction(widgetModel.exposureModeRangeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            updateExposureModeRange(it)
        })
    }

    override fun getIdealDimensionRatioString(): String? {
        return null
    }

    override fun onClick(v: View?) {

        val previousExposureMode: CameraExposureMode = widgetModel.exposureModeProcessor.value
        var exposureMode: CameraExposureMode = CameraExposureMode.UNKNOWN

        when (v?.id) {
            R.id.layout_camera_mode_p -> exposureMode = CameraExposureMode.PROGRAM
            R.id.layout_camera_mode_a -> exposureMode = CameraExposureMode.APERTURE_PRIORITY
            R.id.layout_camera_mode_s -> exposureMode = CameraExposureMode.SHUTTER_PRIORITY
            R.id.layout_camera_mode_m -> exposureMode = CameraExposureMode.MANUAL
            else -> {
                //do nothing
            }
        }

        if (exposureMode == previousExposureMode) {
            return
        }

        updateExposureMode(exposureMode)

        addDisposable(
            widgetModel.setExposureMode(exposureMode)
                .observeOn(SchedulerProvider.ui())
                .subscribe(Action { }, UxErrorHandle.errorConsumer({
                    restoreToCurrentExposureMode()
                }, this.toString(), "setExposureMode: "))
        )
    }

    private fun updateExposureModeRange(range: List<CameraExposureMode>) {
        binding.layoutCameraModeA.isEnabled = rangeContains(range, CameraExposureMode.APERTURE_PRIORITY)
        binding.layoutCameraModeS.isEnabled = rangeContains(range, CameraExposureMode.SHUTTER_PRIORITY)
        binding.layoutCameraModeM.isEnabled = rangeContains(range, CameraExposureMode.MANUAL)
        binding.layoutCameraModeP.isEnabled = rangeContains(range, CameraExposureMode.PROGRAM)
    }

    private fun updateExposureMode(mode: CameraExposureMode) {
        binding.layoutCameraModeA.isSelected = false
        binding.layoutCameraModeS.isSelected = false
        binding.layoutCameraModeM.isSelected = false
        binding.layoutCameraModeP.isSelected = false

        when (mode) {
            CameraExposureMode.PROGRAM -> binding.layoutCameraModeP.isSelected = true
            CameraExposureMode.SHUTTER_PRIORITY -> binding.layoutCameraModeS.isSelected = true
            CameraExposureMode.APERTURE_PRIORITY -> binding.layoutCameraModeA.isSelected = true
            CameraExposureMode.MANUAL -> binding.layoutCameraModeM.isSelected = true
            else -> {
                //do something
            }
        }
    }

    private fun restoreToCurrentExposureMode() {
        updateExposureMode(widgetModel.exposureModeProcessor.value)
    }

    private fun rangeContains(range: List<CameraExposureMode>?, value: CameraExposureMode): Boolean {
        if (range == null) {
            return false
        }
        for (item in range) {
            if (item == value) {
                return true
            }
        }
        return false
    }

    sealed class ModelState
}