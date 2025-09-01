package dji.sampleV5.aircraft.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.navigation.Navigation
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.databinding.FragMopCenterPageBinding

/**
 * Description :
 *
 * @author: Byte.Cai
 *  date : 2023/2/22
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
class MOPCenterFragment : DJIFragment() {

    private var binding: FragMopCenterPageBinding? = null

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        binding = FragMopCenterPageBinding.inflate(inflater, container, false)
        return binding?.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        binding?.btOpenMopDownloadPage?.setOnClickListener {
            Navigation.findNavController(it).navigate(R.id.action_open_down_page)
        }

        binding?.btOpenMopInterfacePage?.setOnClickListener {
            Navigation.findNavController(it).navigate(R.id.action_open_mop_interface_page)

        }

    }
}