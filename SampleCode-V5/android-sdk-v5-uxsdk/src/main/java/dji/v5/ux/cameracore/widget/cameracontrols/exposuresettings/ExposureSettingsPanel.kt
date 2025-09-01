package dji.v5.ux.cameracore.widget.cameracontrols.exposuresettings

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.databinding.UxsdkPanelExposureSettingBinding

/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/10/19
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */
open class ExposureSettingsPanel @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : ConstraintLayoutWidget<ExposureSettingsPanel.ModelState>(context, attrs, defStyleAttr),
    ICameraIndex {

    private lateinit var binding: UxsdkPanelExposureSettingBinding

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkPanelExposureSettingBinding.inflate(LayoutInflater.from(context), this)
    }

    override fun reactToModelChanges() {
        //暂未实现
    }

    override fun getCameraIndex() = binding.exposureSettingWidget.getCameraIndex()

    override fun getLensType() = binding.exposureSettingWidget.getLensType()

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) {
        binding.exposureSettingWidget.updateCameraSource(cameraIndex, lensType)
        binding.isoAndEiSettingWidget.updateCameraSource(cameraIndex, lensType)
    }

    override fun getIdealDimensionRatioString(): String? {
        return null
    }

    sealed class ModelState
}