package dji.sampleV5.aircraft.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import dji.sampleV5.aircraft.databinding.FragUasUsPageBinding
import dji.sampleV5.aircraft.models.UASUAVM

/**
 * Description :美国无人机远程识别示例
 *
 * @author: Byte.Cai
 *  date : 2022/8/2
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
class UASUSFragment : DJIFragment() {
    private val uas: UASUAVM by viewModels()
    private var binding: FragUasUsPageBinding? = null

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        binding = FragUasUsPageBinding.inflate(inflater, container, false)
        return binding?.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        uas.addRemoteIdStatusListener()

        uas.uasRemoteIDStatus.observe(viewLifecycleOwner) {
            binding?.tvUaRidStatus?.text = "RemoteIdStatus:${it}"
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        uas.clearRemoteIdStatusListener()
    }
}