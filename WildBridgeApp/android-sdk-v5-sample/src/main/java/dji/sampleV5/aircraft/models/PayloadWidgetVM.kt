package dji.sampleV5.aircraft.models

import android.util.Log
import androidx.lifecycle.MutableLiveData
import dji.sampleV5.aircraft.data.DJIToastResult
import dji.sdk.keyvalue.value.payload.WidgetValue
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.payload.PayloadCenter
import dji.v5.manager.aircraft.payload.PayloadIndexType
import dji.v5.manager.aircraft.payload.data.PayloadBasicInfo
import dji.v5.manager.aircraft.payload.data.PayloadWidgetInfo
import dji.v5.manager.aircraft.payload.listener.PayloadBasicInfoListener
import dji.v5.manager.aircraft.payload.listener.PayloadWidgetInfoListener


/**
 * Description :
 *
 * @author: Byte.Cai
 *  date : 2022/12/1
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
class PayloadWidgetVM : DJIViewModel() {
    private val TAG = "PayloadWidgetVM"
    private lateinit var payloadIndexType: PayloadIndexType
    // Fetch the live manager map on each access. Caching it at construction (the VM is built
    // in the host activity's onCreate, before any payload has connected) can capture an empty
    // map so no manager is ever found for the target index.
    private val payloadManagerMap get() = PayloadCenter.getInstance().payloadManager
    val payloadBasicInfo = MutableLiveData<PayloadBasicInfo>()
    val payloadWidgetInfo = MutableLiveData<PayloadWidgetInfo>()

    private val payloadBasicInfoListener: PayloadBasicInfoListener = PayloadBasicInfoListener { info -> payloadBasicInfo.postValue(info) }
    private val payloadWidgetInfoListener: PayloadWidgetInfoListener = PayloadWidgetInfoListener {
            info -> payloadWidgetInfo.postValue(info)
    }

    fun setWidgetValue(value: WidgetValue) {
        val map = payloadManagerMap
        val payloadManager = map[payloadIndexType]
        if (payloadManager == null) {
            Log.e(TAG, "setWidgetValue: no payload manager for index=$payloadIndexType (available=${map.keys})")
            return
        }
        payloadManager.setWidgetValue(value, object : CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                sendToastMsg(DJIToastResult.success("setWidgetValue success"))
            }

            override fun onFailure(error: IDJIError) {
                Log.e(TAG, "setWidgetValue failed (type=${value.type} idx=${value.index} val=${value.value}): $error")
                sendToastMsg(DJIToastResult.failed(error.toString()))
            }
        })
    }

    fun initListener(payloadIndexType: PayloadIndexType) {
        this.payloadIndexType = payloadIndexType
        val iPayloadManager = payloadManagerMap[payloadIndexType]
        iPayloadManager?.addPayloadBasicInfoListener(payloadBasicInfoListener)
        iPayloadManager?.addPayloadWidgetInfoListener(payloadWidgetInfoListener)
    }

    fun pullWidgetInfo() {
        val iPayloadManager = payloadManagerMap[payloadIndexType]
        iPayloadManager?.pullWidgetInfoFromPayload(object :CommonCallbacks.CompletionCallback{
            override fun onSuccess() {
                sendToastMsg(DJIToastResult.success("pullWidgetInfoFromPayload success"))
            }

            override fun onFailure(error: IDJIError) {
                sendToastMsg(DJIToastResult.failed(error.toString()))
            }
        })
    }

    override fun onCleared() {
        super.onCleared()
        // payloadIndexType is only set in initListener(); if the VM is cleared before any
        // listener was registered (e.g. activity destroyed without ever calling /send/drop),
        // there is nothing to unregister and the lateinit would otherwise throw.
        if (!::payloadIndexType.isInitialized) return
        val iPayloadManager = payloadManagerMap[payloadIndexType]
        iPayloadManager?.removePayloadWidgetInfoListener(payloadWidgetInfoListener)
        iPayloadManager?.removePayloadBasicInfoListener(payloadBasicInfoListener)
    }

    private fun sendToastMsg(djiToastResult: DJIToastResult) {
        toastResult?.postValue(djiToastResult)
    }
}