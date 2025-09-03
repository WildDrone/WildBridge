package dji.sampleV5.aircraft.models

import android.content.Context
import android.content.Intent
import androidx.core.content.FileProvider
import dji.sampleV5.aircraft.data.DJIToastResult
import dji.sdk.keyvalue.key.RemoteControllerKey
import dji.v5.et.action
import dji.v5.et.create
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DeviceInfoUtil
import dji.v5.utils.common.FileUtils
import java.io.File

/**
 * ClassName : MSDKLogVM
 * Description : Log展示
 * Author : daniel.chen
 * CreateDate : 2022/5/7 12:17 下午
 * Copyright : ©2022 DJI All Rights Reserved.
 */
class APPSilentlyUpgradeVM : DJIViewModel() {

    private val testPackageName = "com.dji.test"
    private val testApkName = "app-debug.apk"

    /**
     * 只适配了M350和Mavic3行业版本的遥控器。
     * 设置以后，对应包名的apk，通过代码安装时，遥控不再需要用户操作。
     * 重启遥控后设置清空。
     */
    fun setAPPSilentlyUpgrade(context: Context) {
        RemoteControllerKey.KeyAPPSilentlyUpgrade.create().action(testPackageName, {
            toastResult?.postValue(DJIToastResult.success(testPackageName))
        }) {
            toastResult?.postValue(DJIToastResult.failed("$it $testPackageName"))
        }
    }

    //本示例，安装包路径: /sdcard/Android/data/你的app的包名/files/你的apk文件名.apk
    //实际路径请按自身需求设置，通过FileProvider安装APK
    //测试apk（app-debug.apk）在sample的assets目录下
    fun installApkWithOutNotice(context: Context) {
        FileUtils.copyAssetsFileIfNeed(context, "apk/$testApkName", File(context.getExternalFilesDir("/"), testApkName))
        val intent = Intent(Intent.ACTION_VIEW)
        intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        val file = File(context.getExternalFilesDir("/"), testApkName)
        val uri = FileProvider.getUriForFile(context, DeviceInfoUtil.getPackageName() + ".fileProvider", file)
        intent.setDataAndType(uri, "application/vnd.android.package-archive")
        context.startActivity(intent)
    }
}