package dji.sampleV5.aircraft

import android.content.Context
import android.util.Log

/**
 * Class Description
 *
 * @author Hoker
 * @date 2022/3/2
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
class DJIAircraftApplication : DJIApplication() {

    override fun attachBaseContext(base: Context?) {
        Log.d("DJIAircraftApp", "attachBaseContext() called")
        try {
            super.attachBaseContext(base)
            Log.d("DJIAircraftApp", "super.attachBaseContext() completed")
            com.cySdkyc.clx.Helper.install(this)
            Log.d("DJIAircraftApp", "Helper.install() completed")
        } catch (e: Exception) {
            Log.e("DJIAircraftApp", "Error in attachBaseContext: ${e.message}", e)
            throw e
        }
    }

    override fun onCreate() {
        Log.d("DJIAircraftApp", "DJIAircraftApplication onCreate() called")
        try {
            super.onCreate()
            Log.d("DJIAircraftApp", "DJIAircraftApplication onCreate() completed successfully")
        } catch (e: Exception) {
            Log.e("DJIAircraftApp", "Error in DJIAircraftApplication onCreate: ${e.message}", e)
            throw e
        }
    }
}