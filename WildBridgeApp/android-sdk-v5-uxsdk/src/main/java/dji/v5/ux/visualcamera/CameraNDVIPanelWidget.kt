package dji.v5.ux.visualcamera

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.databinding.UxsdkPanelNdvlBinding
import dji.v5.ux.databinding.UxsdkPrimaryFlightDisplayWidgetBinding

/**
 * Class Description
 *
 * @author Hoker
 * @date 2022/12/1
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
open class CameraNDVIPanelWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : ConstraintLayoutWidget<Any>(context, attrs, defStyleAttr),
    ICameraIndex {

    private lateinit var binding: UxsdkPanelNdvlBinding
    var mCameraIndex = ComponentIndexType.LEFT_OR_MAIN
    var mLensType = CameraLensType.CAMERA_LENS_ZOOM

    override fun getCameraIndex(): ComponentIndexType {
        return mCameraIndex
    }

    override fun getLensType(): CameraLensType {
        return mLensType
    }

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) {
        mCameraIndex = cameraIndex
        mLensType = lensType
        binding.widgetNdviStreamSelector.updateCameraSource(cameraIndex, lensType)
        binding.widgetSpectralDisplayMode.updateCameraSource(cameraIndex, lensType)
        binding.widgetNdviStreamPaletteBar.updateCameraSource(cameraIndex, lensType)
        binding.widgetNdviStreamPaletteBar.visibility = if (lensType == CameraLensType.CAMERA_LENS_MS_NDVI) VISIBLE else INVISIBLE
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkPanelNdvlBinding.inflate(LayoutInflater.from(context),this,true)
        if (background == null) {
            setBackgroundResource(R.drawable.uxsdk_background_black_rectangle)
        }
    }

    override fun reactToModelChanges() {
        //do nothing
    }

    override fun getIdealDimensionRatioString(): String? = null
}