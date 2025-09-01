package dji.v5.ux.remotecontroller

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.widget.LinearLayout
import dji.v5.utils.common.StringUtils
import dji.v5.ux.R
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.util.ViewUtil
import dji.v5.ux.databinding.UxsdkPanelNdvlBinding
import dji.v5.ux.databinding.UxsdkWidgetRcCheckFrequencyLayoutBinding
import dji.v5.ux.databinding.UxsdkWidgetSettingRcCalibrationLayoutBinding
import dji.v5.ux.remotecontroller.calibration.SmartControllerCalibrationView

/**
 * Description :
 *
 * @author: Byte.Cai
 *  date : 2023/8/18
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
class RCCalibrationWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0,

    ) : ConstraintLayoutWidget<RCPairingWidget.ModelState>(context, attrs, defStyleAttr) {

    private lateinit var binding: UxsdkWidgetSettingRcCalibrationLayoutBinding
    private var mSmartControllerCalibrationView: SmartControllerCalibrationView? = null
    private var sHasShowDialog = false

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkWidgetSettingRcCalibrationLayoutBinding.inflate(LayoutInflater.from(context),this,true)
    }

    override fun reactToModelChanges() {
        initSmartControllerCalibrationView()
    }

    private fun initSmartControllerCalibrationView() {
        binding.rcCalibrationLayout.removeAllViews()
        if (mSmartControllerCalibrationView == null) {
            mSmartControllerCalibrationView =
                LayoutInflater.from(context).inflate(R.layout.uxsdk_setting_ui_rc_smart_controller_calibration, null, false) as SmartControllerCalibrationView
            mSmartControllerCalibrationView?.layoutParams = LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT)
        }
        binding.rcCalibrationLayout.addView(mSmartControllerCalibrationView)
        if (!sHasShowDialog) {
            val content = StringUtils.getResStr(R.string.uxsdk_setting_ui_rc_cele_tip)
            ViewUtil.showToast(context, content)
            sHasShowDialog = true
        }
    }
}