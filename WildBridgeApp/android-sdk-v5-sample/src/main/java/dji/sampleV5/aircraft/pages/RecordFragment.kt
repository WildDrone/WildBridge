package dji.sampleV5.aircraft.pages

import android.content.res.ColorStateList
import android.graphics.PorterDuff
import android.os.Bundle
import android.os.SystemClock
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.core.content.ContextCompat
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.databinding.FragRecordBinding
import dji.sampleV5.aircraft.models.MegaphoneVM
import dji.v5.utils.common.ContextUtil

/**
 * Description : 录音Fragment
 * Author : daniel.chen
 * CreateDate : 2022/1/17 2:41 下午
 * Copyright : ©2021 DJI All Rights Reserved.
 */
class RecordFragment : DJIFragment() {
    private val megaphoneVM: MegaphoneVM by activityViewModels()
    private var binding: FragRecordBinding? = null
    private var recordStarted: Boolean = false

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        binding = FragRecordBinding.inflate(inflater, container, false)
        return binding?.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initBtnListener()
    }

    private fun initBtnListener() {
        //点击触发录音
        binding?.btnRecord?.setOnClickListener {
            if (!recordStarted) {
                megaphoneVM.startRecord()
                binding?.chronometer?.base = SystemClock.elapsedRealtime()
                binding?.chronometer?.start()
                val colorStateList: ColorStateList? = ContextCompat.getColorStateList(ContextUtil.getContext(), R.color.red)
                binding?.btnRecord?.imageTintMode = PorterDuff.Mode.SRC_ATOP
                binding?.btnRecord?.imageTintList = colorStateList
                recordStarted = true
            } else {
                megaphoneVM.stopRecord()
                binding?.chronometer?.stop()
                binding?.chronometer?.base = SystemClock.elapsedRealtime()
                val colorStateList: ColorStateList? = ContextCompat.getColorStateList(ContextUtil.getContext(), R.color.green)
                binding?.btnRecord?.imageTintMode = PorterDuff.Mode.SRC_ATOP
                binding?.btnRecord?.imageTintList = colorStateList
                recordStarted = false
            }
        }

        binding?.cbPlay?.setOnCheckedChangeListener { _, isChecked ->
            megaphoneVM.isQuickPlay = isChecked
        }
    }
}