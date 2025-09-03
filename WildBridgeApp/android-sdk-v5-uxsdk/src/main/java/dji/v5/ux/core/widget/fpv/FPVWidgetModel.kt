/*
 * Copyright (c) 2018-2020 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
package dji.v5.ux.core.widget.fpv

import android.view.Surface
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.FlightControllerKey
import dji.sdk.keyvalue.value.camera.CameraType
import dji.sdk.keyvalue.value.camera.CameraVideoStreamSourceType
import dji.sdk.keyvalue.value.camera.VideoResolutionFrameRate
import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.et.create
import dji.v5.et.createCamera
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.interfaces.ICameraStreamManager
import dji.v5.utils.common.LogUtils
import dji.v5.utils.common.StringUtils
import dji.v5.ux.R
import dji.v5.ux.core.base.DJISDKModel
import dji.v5.ux.core.base.ICameraIndex
import dji.v5.ux.core.base.WidgetModel
import dji.v5.ux.core.communication.ObservableInMemoryKeyedStore
import dji.v5.ux.core.module.FlatCameraModule
import dji.v5.ux.core.util.CameraUtil
import dji.v5.ux.core.util.DataProcessor
import dji.v5.ux.core.util.UxErrorHandle
import io.reactivex.rxjava3.core.Flowable

/**
 * Widget Model for the [FPVWidget] used to define
 * the underlying logic and communication
 */
class FPVWidgetModel(
    djiSdkModel: DJISDKModel,
    keyedStore: ObservableInMemoryKeyedStore,
    private val flatCameraModule: FlatCameraModule,
) : WidgetModel(djiSdkModel, keyedStore), ICameraIndex {

    private var currentLensType = CameraLensType.CAMERA_LENS_DEFAULT
    private val streamSourceCameraTypeProcessor = DataProcessor.create(CameraVideoStreamSourceType.UNKNOWN)
    private val resolutionAndFrameRateProcessor: DataProcessor<VideoResolutionFrameRate> = DataProcessor.create(VideoResolutionFrameRate())
    private val cameraTypeProcessor: DataProcessor<CameraType> = DataProcessor.create(CameraType.NOT_SUPPORTED)
    private val isMotorOnProcessor: DataProcessor<Boolean> = DataProcessor.create(false)
    val displayMsgProcessor: DataProcessor<String> = DataProcessor.create("")
    val cameraSideProcessor: DataProcessor<String> = DataProcessor.create("")
    private val videoViewChangedProcessor: DataProcessor<Boolean> = DataProcessor.create(false)
    var streamSourceListener: FPVStreamSourceListener? = null
    private var cameraType: CameraType = CameraType.NOT_SUPPORTED

    /**
     * The current camera index. This value should only be used for video size calculation.
     * To get the camera side, use [FPVWidgetModel.cameraSide] instead.
     */
    private var currentCameraIndex: ComponentIndexType = ComponentIndexType.UNKNOWN

    /**
     * Get whether the video view has changed
     */
    @get:JvmName("hasVideoViewChanged")
    val hasVideoViewChanged: Flowable<Boolean>
        get() = videoViewChangedProcessor.toFlowable()

    init {
        addModule(flatCameraModule)
    }

    override fun getCameraIndex(): ComponentIndexType {
        return currentCameraIndex
    }

    override fun getLensType(): CameraLensType {
        return currentLensType
    }

    override fun updateCameraSource(cameraIndex: ComponentIndexType, lensType: CameraLensType) {
        if (currentCameraIndex != cameraIndex) {
            currentCameraIndex = cameraIndex
            restart()
        }
    }

    //region Lifecycle
    override fun inSetup() {
        val videoViewChangedConsumer = { _: Any -> videoViewChangedProcessor.onNext(true) }
        bindDataProcessor(CameraKey.KeyCameraVideoStreamSource.create(currentCameraIndex), streamSourceCameraTypeProcessor) {
            currentLensType = when (it) {
                CameraVideoStreamSourceType.WIDE_CAMERA -> CameraLensType.CAMERA_LENS_WIDE
                CameraVideoStreamSourceType.ZOOM_CAMERA -> CameraLensType.CAMERA_LENS_ZOOM
                CameraVideoStreamSourceType.INFRARED_CAMERA -> CameraLensType.CAMERA_LENS_THERMAL
                CameraVideoStreamSourceType.NDVI_CAMERA -> CameraLensType.CAMERA_LENS_MS_NDVI
                CameraVideoStreamSourceType.MS_G_CAMERA -> CameraLensType.CAMERA_LENS_MS_G
                CameraVideoStreamSourceType.MS_R_CAMERA -> CameraLensType.CAMERA_LENS_MS_R
                CameraVideoStreamSourceType.MS_RE_CAMERA -> CameraLensType.CAMERA_LENS_MS_RE
                CameraVideoStreamSourceType.MS_NIR_CAMERA -> CameraLensType.CAMERA_LENS_MS_NIR
                CameraVideoStreamSourceType.RGB_CAMERA -> CameraLensType.CAMERA_LENS_RGB
                else -> CameraLensType.CAMERA_LENS_DEFAULT
            }
            sourceUpdate()
        }

        bindDataProcessor(FlightControllerKey.KeyAreMotorsOn.create(), isMotorOnProcessor) {
            updateCameraDisplay()
        }

        bindDataProcessor(CameraKey.KeyCameraType.create(currentCameraIndex), cameraTypeProcessor) {
            cameraType = it
            updateCameraDisplay()
        }

        bindDataProcessor(CameraKey.KeyVideoResolutionFrameRate.createCamera(currentCameraIndex, currentLensType), resolutionAndFrameRateProcessor)

        addDisposable(
            flatCameraModule.cameraModeDataProcessor.toFlowable()
                .doOnNext(videoViewChangedConsumer)
                .subscribe({ }, UxErrorHandle.logErrorConsumer(tag, "camera mode: "))
        )
        sourceUpdate()
    }

    override fun inCleanup() {
        currentLensType = CameraLensType.CAMERA_LENS_DEFAULT
    }

    private fun sourceUpdate() {
        onStreamSourceUpdated()
        updateCameraDisplay()
    }

    private fun updateCameraDisplay() {
        var msg = ""
        if (!CameraUtil.isFPVTypeView(currentCameraIndex)) {
            msg = cameraType.name
        }
        if (currentCameraIndex == ComponentIndexType.VISION_ASSIST) {
            if (!isMotorOnProcessor.value) {
                msg = StringUtils.getResStr(R.string.uxsdk_assistant_video_empty_text)
            }
        }
        if (currentLensType != CameraLensType.CAMERA_LENS_DEFAULT && currentLensType != CameraLensType.UNKNOWN) {
            msg = msg + "_" + currentLensType.name
        }
        displayMsgProcessor.onNext(msg)
        cameraSideProcessor.onNext(currentCameraIndex.name)
    }

    public override fun updateStates() {
        //无需实现
    }

    fun putCameraStreamSurface(
        surface: Surface,
        width: Int,
        height: Int,
        scaleType: ICameraStreamManager.ScaleType
    ) {
        MediaDataCenter.getInstance().cameraStreamManager.putCameraStreamSurface(currentCameraIndex, surface, width, height, scaleType)
    }

    fun removeCameraStreamSurface(surface: Surface) {
        MediaDataCenter.getInstance().cameraStreamManager.removeCameraStreamSurface(surface)
    }

    fun enableVisionAssist() {
        MediaDataCenter.getInstance().cameraStreamManager.enableVisionAssist(true, null)
    }

    private fun onStreamSourceUpdated() {
        streamSourceListener?.onStreamSourceUpdated(currentCameraIndex, currentLensType)
    }
}