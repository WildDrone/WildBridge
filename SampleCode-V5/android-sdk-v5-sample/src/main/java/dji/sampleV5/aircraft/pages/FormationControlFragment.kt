package dji.sampleV5.aircraft.pages

import android.annotation.SuppressLint
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.controller.DroneController
import dji.sampleV5.aircraft.controller.FormationController
import dji.sampleV5.aircraft.controller.DroneRole
import dji.sampleV5.aircraft.databinding.FragmentFormationControlBinding
import dji.sampleV5.aircraft.models.BasicAircraftControlVM
import dji.sampleV5.aircraft.models.VirtualStickVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.key.BatteryKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.common.LocationCoordinate3D
import dji.v5.et.create
import dji.v5.et.get
import java.text.DecimalFormat

class FormationControlFragment : Fragment() {

    private var _binding: FragmentFormationControlBinding? = null
    private val binding get() = _binding!!

    private val basicAircraftControlVM: BasicAircraftControlVM by activityViewModels()
    private val virtualStickVM: VirtualStickVM by activityViewModels()

    private val mainHandler = Handler(Looper.getMainLooper())
    private var telemetryUpdateRunnable: Runnable? = null

    private val location3DKey = FlightControllerKey.KeyAircraftLocation3D.create()
    private val compassHeadKey = FlightControllerKey.KeyCompassHeading.create()
    private val batteryPercentKey = BatteryKey.KeyChargeRemainingInPercent.create()

    private val decimalFormat = DecimalFormat("#.######")

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentFormationControlBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        // Initialize DroneController
        DroneController.init(basicAircraftControlVM, virtualStickVM)

        setupClickListeners()
        startTelemetryUpdates()
    }

    private fun setupClickListeners() {
        // Role selection
        binding.btnLeader.setOnClickListener {
            FormationController.initAsLeader()
            updateRoleUI(DroneRole.LEADER)
        }

        binding.btnFollower.setOnClickListener {
            updateRoleUI(DroneRole.FOLLOWER)
            binding.layoutLeaderIp.visibility = View.VISIBLE
        }

        // Connection
        binding.btnConnect.setOnClickListener {
            val leaderIp = binding.etLeaderIp.text.toString().trim()
            if (leaderIp.isNotEmpty()) {
                FormationController.initAsFollower(leaderIp)
                binding.layoutLeaderIp.visibility = View.GONE
            } else {
                ToastUtils.showToast("Please enter leader IP address")
            }
        }

        // Formation controls
        binding.btnStartFormation.setOnClickListener {
            FormationController.startFormation()
        }

        binding.btnStopFormation.setOnClickListener {
            FormationController.stopFormation()
        }

        binding.btnEmergencyStop.setOnClickListener {
            FormationController.emergencyStop()
        }

        // Manual override controls
        binding.btnTakeoff.setOnClickListener {
            DroneController.startTakeOff()
        }

        binding.btnLand.setOnClickListener {
            DroneController.startLanding()
        }

        binding.btnRth.setOnClickListener {
            DroneController.startReturnToHome()
        }

        binding.btnEnableVirtualStick.setOnClickListener {
            DroneController.enableVirtualStick()
        }

        binding.btnDisableVirtualStick.setOnClickListener {
            DroneController.disableVirtualStick()
        }
    }

    private fun updateRoleUI(role: DroneRole) {
        when (role) {
            DroneRole.LEADER -> {
                binding.btnLeader.isEnabled = false
                binding.btnFollower.isEnabled = true
                binding.btnStartFormation.isEnabled = true
                binding.btnStopFormation.isEnabled = true
                binding.layoutLeaderIp.visibility = View.GONE
            }
            DroneRole.FOLLOWER -> {
                binding.btnLeader.isEnabled = true
                binding.btnFollower.isEnabled = false
                binding.btnStartFormation.isEnabled = true
                binding.btnStopFormation.isEnabled = true
            }
            DroneRole.NONE -> {
                binding.btnLeader.isEnabled = true
                binding.btnFollower.isEnabled = true
                binding.btnStartFormation.isEnabled = false
                binding.btnStopFormation.isEnabled = false
                binding.layoutLeaderIp.visibility = View.GONE
            }
        }
    }

    private fun startTelemetryUpdates() {
        telemetryUpdateRunnable = object : Runnable {
            override fun run() {
                updateTelemetryDisplay()
                mainHandler.postDelayed(this, 500) // Update every 500ms
            }
        }
        mainHandler.post(telemetryUpdateRunnable!!)
    }

    @SuppressLint("SetTextI18n")
    private fun updateTelemetryDisplay() {
        try {
            // Update connection and formation status
            binding.tvConnectionStatus.text = FormationController.getConnectionStatus()
            binding.tvFormationStatus.text = FormationController.getFormationStatus()

            // Update my drone telemetry
            val myPosition = location3DKey.get(LocationCoordinate3D(0.0, 0.0, 0.0))
            val myHeading = compassHeadKey.get(0.0)
            val myBattery = batteryPercentKey.get(0)

            binding.tvMyTelemetry.text = """
                Position: ${decimalFormat.format(myPosition.latitude)}, ${decimalFormat.format(myPosition.longitude)}
                Altitude: ${decimalFormat.format(myPosition.altitude)}m
                Heading: ${decimalFormat.format(myHeading)}°
                Battery: ${myBattery}%
            """.trimIndent()

            // Update other drone telemetry
            val otherDrone = FormationController.getOtherDroneState()
            if (otherDrone != null) {
                val distance = DroneController.calculateDistance(
                    myPosition.latitude, myPosition.longitude,
                    otherDrone.position.latitude, otherDrone.position.longitude
                )

                binding.tvOtherTelemetry.text = """
                    Position: ${decimalFormat.format(otherDrone.position.latitude)}, ${decimalFormat.format(otherDrone.position.longitude)}
                    Altitude: ${decimalFormat.format(otherDrone.position.altitude)}m
                    Heading: ${decimalFormat.format(otherDrone.heading)}°
                    Battery: ${otherDrone.battery}%
                    Distance: ${decimalFormat.format(distance)}m
                """.trimIndent()
            } else {
                binding.tvOtherTelemetry.text = """
                    Position: --
                    Altitude: --
                    Heading: --
                    Battery: --%
                    Distance: --
                """.trimIndent()
            }

        } catch (e: Exception) {
            // Handle any telemetry read errors silently
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        telemetryUpdateRunnable?.let { mainHandler.removeCallbacks(it) }
        FormationController.cleanup()
        _binding = null
    }
}
