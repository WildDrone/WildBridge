package dji.v5.ux.core.widget.hsi

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.databinding.UxsdkFpvViewHorizontalSituationIndicatorBinding

/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/11/25
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */
open class HorizontalSituationIndicatorWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,
) : ConstraintLayoutWidget<HorizontalSituationIndicatorWidget.ModelState>(context, attrs, defStyleAttr),
    ICameraIndex {

    private lateinit var binding: UxsdkFpvViewHorizontalSituationIndicatorBinding

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkFpvViewHorizontalSituationIndicatorBinding.inflate(LayoutInflater.from(context), this, true)
    }

    override fun reactToModelChanges() {
//        do nothing
    }

    override fun getIdealDimensionRatioString(): String? {
        return null
    }

    fun setSimpleModeEnable(isEnable: Boolean) {
        binding.pfdHsiSpeedDisplay.visibility = if (isEnable) VISIBLE else GONE
        binding.pfdHsiAttitudeDisplay.visibility = if (isEnable) VISIBLE else GONE
        binding.pfdHsiGimbalPitchDisplay.visibility = if (isEnable) VISIBLE else GONE
    }

    sealed class ModelState

    override fun getCameraIndex(): ComponentIndexType {
        return binding.pfdHsiGimbalPitchDisplay.getCameraIndex()
    }

    override fun getLensType(): CameraLensType {
        return binding.pfdHsiGimbalPitchDisplay.getLensType()

    }

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) {
        binding.pfdHsiGimbalPitchDisplay.updateCameraSource(cameraIndex, lensType)
    }
}