package dji.sampleV5.aircraft

import android.content.Intent
import android.os.Handler
import android.os.Looper
import androidx.appcompat.app.AlertDialog
import dji.v5.common.utils.GeoidManager
import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.ux.core.communication.DefaultGlobalPreferences
import dji.v5.ux.core.communication.GlobalPreferencesManager
import dji.v5.ux.core.util.UxSharedPreferencesUtil
import dji.v5.ux.sample.showcase.widgetlist.WidgetsActivity

/**
 * Class Description
 *
 * @author Hoker
 * @date 2022/2/14
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
class DJIAircraftMainActivity : DJIMainActivity() {

    private val recoveryHandler = Handler(Looper.getMainLooper())
    private var noDronePrompt: AlertDialog? = null
    private var noDronePromptShown = false

    override fun prepareUxActivity() {
        UxSharedPreferencesUtil.initialize(this)
        GlobalPreferencesManager.initialize(DefaultGlobalPreferences(this))
        GeoidManager.getInstance().init(this)

        enableDefaultLayout(WildBridgeDefaultLayoutActivity::class.java)
        enableWidgetList(WidgetsActivity::class.java)
    }

    override fun prepareTestingToolsActivity() {
        enableTestingTools(AircraftTestingToolsActivity::class.java)
    }

    override fun onResume() {
        super.onResume()
        scheduleNoDroneRecoveryPrompt()
    }

    override fun onPause() {
        recoveryHandler.removeCallbacksAndMessages(null)
        super.onPause()
    }

    override fun onDestroy() {
        noDronePrompt?.dismiss()
        noDronePrompt = null
        recoveryHandler.removeCallbacksAndMessages(null)
        super.onDestroy()
    }

    private fun scheduleNoDroneRecoveryPrompt() {
        if (noDronePromptShown || noDronePrompt?.isShowing == true) return
        recoveryHandler.removeCallbacksAndMessages(null)
        recoveryHandler.postDelayed({
            if (!isFinishing && !isDestroyed && shouldOfferNoDroneRecovery()) {
                showNoDroneRecoveryPrompt()
            }
        }, NO_DRONE_PROMPT_DELAY_MS)
    }

    private fun shouldOfferNoDroneRecovery(): Boolean {
        return try {
            when (ProductKey.KeyProductType.create().get(ProductType.UNKNOWN)) {
                ProductType.UNKNOWN, ProductType.UNRECOGNIZED -> true
                else -> false
            }
        } catch (_: Throwable) {
            true
        }
    }

    private fun showNoDroneRecoveryPrompt() {
        noDronePromptShown = true
        noDronePrompt = AlertDialog.Builder(this)
            .setTitle("No drone detected")
            .setMessage("WildBridge still does not see a drone. Make sure the drone is on, the controller is on, the controller USB-C port is connected properly, and DJI Fly or any other DJI app is not running in the background, then reopen WildBridge to retry detection.")
            .setPositiveButton("Reopen WildBridge") { _, _ ->
                reopenWildBridge()
            }
            .setNegativeButton("Keep waiting", null)
            .setOnDismissListener { noDronePrompt = null }
            .show()
    }

    private fun reopenWildBridge() {
        val restartIntent = Intent(this, DJIAircraftMainActivity::class.java).apply {
            addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK)
        }
        startActivity(restartIntent)
        finishAffinity()
    }

    companion object {
        private const val NO_DRONE_PROMPT_DELAY_MS = 12_000L
    }
}