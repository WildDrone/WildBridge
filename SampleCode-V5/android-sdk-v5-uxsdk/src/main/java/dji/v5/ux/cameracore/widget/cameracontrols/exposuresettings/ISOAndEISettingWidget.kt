package dji.v5.ux.cameracore.widget.cameracontrols.exposuresettings

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import android.view.View
import dji.sdk.keyvalue.value.camera.CameraExposureMode
import dji.sdk.keyvalue.value.camera.CameraExposureSettings
import dji.sdk.keyvalue.value.camera.CameraISO
import dji.sdk.keyvalue.value.camera.EIType
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.SchedulerProvider
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.ui.HorizontalSeekBar
import dji.v5.ux.core.util.AudioUtil
import dji.v5.ux.core.util.CameraUtil
import dji.v5.ux.core.util.UxErrorHandle
import dji.v5.ux.databinding.UxsdkWidgetIsoEiSettingBinding
import io.reactivex.rxjava3.functions.Action

/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/11/2
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */
open class ISOAndEISettingWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : ConstraintLayoutWidget<ISOAndEISettingWidget.ModelState>(context, attrs, defStyleAttr),
    View.OnClickListener, HorizontalSeekBar.OnSeekBarChangeListener, ICameraIndex {

    private val LOCKED_ISO_VALUE = "500"

    private var isISOAutoSelected = false
    private var isISOAutoSupported = false
    private var isISOSeekBarEnabled = false
    private val isISOLocked = false
    private var isSeekBarTracking = false
    private var uiCameraISO = 0

    //去掉auto
    private var uiIsoValueArray: Array<CameraISO?> = arrayOf()
    private var eiValueArray: IntArray = intArrayOf()

    private lateinit var binding: UxsdkWidgetIsoEiSettingBinding

    private val widgetModel by lazy {
        ISOAndEISettingModel(DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance())
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkWidgetIsoEiSettingBinding.inflate(LayoutInflater.from(context), this)
    }

    override fun reactToModelChanges() {

        // ISO part
        addReaction(widgetModel.exposureModeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            onExposureModeUpdated(it)
            updateISOEnableStatus()
        })
        addReaction(widgetModel.ISOProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            onISOUpdated(it)
        })
        addReaction(widgetModel.ISORangeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            updateISORangeValue(it.toTypedArray())
            updateISOEnableStatus()
            updateISORangeUI()
        })
        addReaction(widgetModel.exposureSettingsProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            val exposureParameters = it as CameraExposureSettings
            uiCameraISO = exposureParameters.iso
            updateISORangeUI()
        })

        // EI part
        addReaction(widgetModel.eiValueProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {

        })
        addReaction(widgetModel.eiValueRangeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {

        })
        addReaction(widgetModel.eiRecommendedValueProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {

        })

        // mode
        addReaction(widgetModel.exposureSensitivityModeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            updateWidgetUI()
        })
        addReaction(widgetModel.cameraModeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            updateWidgetUI()
        })
        addReaction(widgetModel.flatCameraModeProcessor.toFlowable().observeOn(SchedulerProvider.ui()).subscribe {
            updateWidgetUI()
        })
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()

        // Initialize ISO array
        val res = context.resources
        val valueArray = res.getIntArray(R.array.uxsdk_iso_values)
        uiIsoValueArray = arrayOfNulls(valueArray.size)

        if (!isInEditMode) {
            for (i in valueArray.indices) {
                uiIsoValueArray[i] = CameraISO.find(valueArray[i])
            }
            updateISORangeValue(uiIsoValueArray)
        }

        // ISO seekBar
        isISOSeekBarEnabled = false
        binding.seekbarIso.progress = 0
        binding.seekbarIso.enable(false)
        binding.seekbarIso.addOnSeekBarChangeListener(this)
        binding.seekbarIso.isBaselineVisibility = false
        binding.seekbarIso.setMinValueVisibility(true)
        binding.seekbarIso.setMaxValueVisibility(true)
        binding.seekbarIso.setMinusVisibility(false)
        binding.seekbarIso.setPlusVisibility(false)
        binding.buttonIsoAuto.setOnClickListener(this)

        // EI seekBar
        binding.seekbarEi.addOnSeekBarChangeListener(this)
        binding.seekbarEi.visibility = GONE
        binding.seekbarEi.setMinValueVisibility(true)
        binding.seekbarEi.setMaxValueVisibility(true)
        binding.seekbarEi.setMinusVisibility(false)
        binding.seekbarEi.setPlusVisibility(false)

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

    override fun getCameraIndex() = widgetModel.getCameraIndex()

    override fun getLensType() = widgetModel.getLensType()

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) = widgetModel.updateCameraSource(cameraIndex, lensType)

    override fun getIdealDimensionRatioString(): String? {
        return null
    }

    override fun onProgressChanged(view: HorizontalSeekBar, progress: Int, isFromUI: Boolean) {
        if (view == binding.seekbarIso) {
            if (isISOLocked) {
                binding.seekbarIso.text = LOCKED_ISO_VALUE
            } else {
                if (uiIsoValueArray.isNotEmpty()) {
                    uiCameraISO = CameraUtil.convertISOToInt(uiIsoValueArray[progress])
                    binding.seekbarIso.text = uiCameraISO.toString()
                }
            }
        } else {
            if (progress < eiValueArray.size) {
                binding.seekbarEi.text = eiValueArray[progress].toString()
            }
        }
    }

    override fun onStartTrackingTouch(view: HorizontalSeekBar, progress: Int) {
        isSeekBarTracking = true
    }

    override fun onStopTrackingTouch(view: HorizontalSeekBar, progress: Int) {
        isSeekBarTracking = false
        AudioUtil.playSoundInBackground(context, R.raw.uxsdk_camera_ev_center)
        if (view == binding.seekbarIso) {
            if (uiIsoValueArray.isNotEmpty()) {
                val newISO = uiIsoValueArray[progress]
                newISO?.let {
                    updateISOToCamera(it)
                }
            }
        } else {
            if (progress < eiValueArray.size) {
                updateEIToCamera(eiValueArray[progress])
            }
        }
    }

    override fun onPlusClicked(view: HorizontalSeekBar) {
        //暂未实现
    }

    override fun onMinusClicked(view: HorizontalSeekBar) {
        //暂未实现
    }

    override fun onClick(v: View?) {
        if (v == binding.buttonIsoAuto) {
            isISOAutoSelected = !isISOAutoSelected
            setAutoISO(isISOAutoSelected)
        }
    }

    private fun updateWidgetUI() {
        if (widgetModel.isRecordVideoEIMode()) {
            binding.textviewIsoTitle.setText(R.string.uxsdk_camera_ei)
            binding.seekbarIsoLayout.visibility = GONE
            binding.seekbarEi.visibility = VISIBLE
        } else {
            binding.textviewIsoTitle.setText(R.string.uxsdk_camera_exposure_iso_title)
            binding.seekbarIsoLayout.visibility = VISIBLE
            binding.seekbarEi.visibility = GONE
        }
    }

    private fun onISOUpdated(iso: CameraISO) {
        if (iso == CameraISO.ISO_FIXED) {
            updateISOLocked()
        }
    }

    private fun onExposureModeUpdated(exposureMode: CameraExposureMode) {
        if (exposureMode != CameraExposureMode.MANUAL) {
            isISOAutoSelected = true
            setAutoISO(isISOAutoSelected)
        } else {
            isISOAutoSelected = false
        }

    }

    private fun updateISORangeValue(array: Array<CameraISO?>) {
        isISOAutoSupported = checkAutoISO(array)
        val newISOValues: Array<CameraISO?> = if (isISOAutoSupported) {
            arrayOfNulls(array.size - 1)
        } else {
            arrayOfNulls(array.size)
        }

        // remove the auto value
        var i = 0
        var j = 0
        while (i < array.size) {
            if (array[i] != CameraISO.ISO_AUTO) {
                newISOValues[j] = array[i]
                j++
            }
            i++
        }
        uiIsoValueArray = newISOValues
    }

    private fun updateISORangeUI() {
        // Workaround where ISO range updates to single value in AUTO mode
        if (uiIsoValueArray.isNotEmpty()) {
            val minCameraISO = CameraUtil.convertISOToInt(uiIsoValueArray[0])
            binding.seekbarIso.setMinValueText(minCameraISO.toString())
            val maxCameraISO = CameraUtil.convertISOToInt(uiIsoValueArray[uiIsoValueArray.size - 1])
            binding.seekbarIso.setMaxValueText(maxCameraISO.toString())
            binding.seekbarIso.max = uiIsoValueArray.size - 1
            isISOSeekBarEnabled = true
            updateISOValue(uiIsoValueArray, uiCameraISO)
            // Auto button has relationship with ISO range, so need update this button here.
            updateAutoISOButton()
        } else {
            isISOSeekBarEnabled = false
        }
    }

    private fun updateISOEnableStatus() {
        binding.seekbarIso.enable(!isISOAutoSelected && isISOSeekBarEnabled)
    }

    private fun checkAutoISO(array: Array<CameraISO?>): Boolean {
        for (iso in array) {
            if (iso == CameraISO.ISO_AUTO) {
                return true
            }
        }
        return false
    }

    private fun updateISOValue(array: Array<CameraISO?>, value: Int) {
        val progress: Int = getISOIndex(array, value)
        binding.seekbarIso.progress = progress
    }

    private fun updateAutoISOButton() {
        if (isISOAutoSupported && isISOSeekBarEnabled && !widgetModel.isRecordVideoEIMode() && CameraUtil.isAutoISOSupportedByProduct()) {
            binding.buttonIsoAuto.visibility = VISIBLE
        } else {
            binding.buttonIsoAuto.visibility = GONE
        }
    }

    private fun getISOIndex(array: Array<CameraISO?>, isoValue: Int): Int {
        var index = -1
        val iso = CameraUtil.convertIntToISO(isoValue)
        for (i in array.indices) {
            if (iso == array[i]) {
                index = i
                break
            }
        }
        return index
    }

    private fun setAutoISO(isAuto: Boolean) {
        var newISO: CameraISO? = null
        if (isAuto) {
            newISO = CameraISO.ISO_AUTO
        } else {
            if (binding.seekbarIso.progress < uiIsoValueArray.size) {
                newISO = uiIsoValueArray[binding.seekbarIso.progress]
            }
        }
        newISO?.let {
            updateISOToCamera(it)
        }
    }

    private fun updateISOToCamera(iso: CameraISO) {
        addDisposable(
            widgetModel.setISO(iso).observeOn(SchedulerProvider.ui()).subscribe(Action { }, UxErrorHandle.errorConsumer({
                binding.seekbarIso.restorePreviousProgress()
            }, this.toString(), "updateISOToCamera: "))
        )
    }

    private fun updateEIToCamera(ei: Int) {
        addDisposable(
            widgetModel.setEI(EIType.find(ei)).observeOn(SchedulerProvider.ui()).subscribe(Action { }, UxErrorHandle.errorConsumer({
                binding.seekbarIso.restorePreviousProgress()
            }, this.toString(), "updateEIToCamera: "))
        )
    }

    // By referring to DJIGo4 in both iOS and Android version
    // Showing the ISO_FIXED  as locked value 500
    private fun updateISOLocked() {
        binding.buttonIsoAuto.visibility = GONE
        binding.seekbarIso.enable(false)
        binding.seekbarIso.progress = binding.seekbarIso.max / 2 - 1
    }

    sealed class ModelState
}