package dji.sampleV5.aircraft.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.databinding.FragAppSilentlyUpgradePageBinding
import dji.sampleV5.aircraft.models.APPSilentlyUpgradeVM

class APPSilentlyUpgradeFragment : DJIFragment() {
    private val vm: APPSilentlyUpgradeVM by activityViewModels()
    private var binding: FragAppSilentlyUpgradePageBinding? = null

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        binding = FragAppSilentlyUpgradePageBinding.inflate(inflater, container, false)
        return binding?.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        binding?.btnSilentlyUpgradePackage?.setOnClickListener {
            vm.setAPPSilentlyUpgrade(requireContext())
        }
        binding?.btnInstallTestApp?.setOnClickListener {
            vm.installApkWithOutNotice(requireContext())
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        binding = null
    }
}