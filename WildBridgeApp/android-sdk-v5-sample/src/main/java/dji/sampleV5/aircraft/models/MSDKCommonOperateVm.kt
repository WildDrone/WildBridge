package dji.sampleV5.aircraft.models

import android.content.Context
import android.content.Intent
import androidx.core.content.FileProvider
import androidx.lifecycle.MutableLiveData
import dji.sampleV5.aircraft.data.DJIToastResult
import dji.sampleV5.aircraft.data.FragmentPageItemList
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.ldm.LDMExemptModule
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.manager.SDKManager
import dji.v5.manager.ldm.LDMManager
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DeviceInfoUtil.getPackageName
import dji.v5.utils.common.FileUtils
import java.io.File


/**
 * Class Description
 *
 * @author Hoker
 * @date 2021/5/10
 *
 * Copyright (c) 2021, DJI All Rights Reserved.
 */
class MSDKCommonOperateVm : DJIViewModel() {

    val mainPageInfoList = MutableLiveData<LinkedHashSet<FragmentPageItemList>>()

    fun loaderItem(itemList: LinkedHashSet<FragmentPageItemList>) {
        mainPageInfoList.postValue(itemList)
    }

    fun enableLDM(context: Context, callback: CommonCallbacks.CompletionCallback, ldmExemptModuleList: Array<LDMExemptModule?>) {
        LDMManager.getInstance().enableLDM(context, callback, * ldmExemptModuleList)
    }

    fun disableLDM(callback: CommonCallbacks.CompletionCallback) {
        LDMManager.getInstance().disableLDM(callback);
    }

    fun registerApp() {
        SDKManager.getInstance().registerApp()
    }
}