package dji.v5.ux.core.widget.hsi

import android.content.Context
import android.util.AttributeSet
import android.view.LayoutInflater
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.widget.ConstraintLayoutWidget
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.databinding.UxsdkPrimaryFlightDisplayWidgetBinding
import io.reactivex.rxjava3.android.schedulers.AndroidSchedulers

/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/11/25
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */
open class PrimaryFlightDisplayWidget @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : ConstraintLayoutWidget<PrimaryFlightDisplayWidget.ModelState>(context, attrs, defStyleAttr) {

    private lateinit var binding: UxsdkPrimaryFlightDisplayWidgetBinding

    private val widgetModel by lazy {
        PrimaryFlightDisplayModel(DJISDKModel.getInstance(), ObservableInMemoryKeyedStore.getInstance())
    }

    override fun initView(context: Context, attrs: AttributeSet?, defStyleAttr: Int) {
        binding = UxsdkPrimaryFlightDisplayWidgetBinding.inflate(LayoutInflater.from(context),this,true)
    }

    override fun reactToModelChanges() {
        addDisposable(widgetModel.velocityProcessor.toFlowable().observeOn(AndroidSchedulers.mainThread()).subscribe {
            binding.fpvAttitude.setSpeedX(it.x.toFloat())
            binding.fpvAttitude.setSpeedY(it.y.toFloat())
            binding.fpvAttitude.setSpeedZ(it.z.toFloat())

        })
        addDisposable(widgetModel.aircraftAttitudeProcessor.toFlowable().observeOn(AndroidSchedulers.mainThread()).subscribe {
            binding.fpvAttitude.setPitch(it.pitch.toFloat())
            binding.fpvAttitude.setRoll(it.roll.toFloat())
            binding.fpvAttitude.setYaw(it.yaw.toFloat())

        })
        setVideoViewSize(1440, 1080)
    }

    //可以通过fpvWidget获取真实的video长宽比
    private fun setVideoViewSize(videoViewWidth: Int, videoViewHeight: Int) {
        binding.fpvAttitude.setVideoViewSize(videoViewWidth, videoViewHeight)
    }

    override fun getIdealDimensionRatioString(): String? {
        return null
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

    sealed class ModelState
}