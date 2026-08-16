package dji.sampleV5.aircraft.models

import androidx.lifecycle.MutableLiveData
import dji.sampleV5.aircraft.R
import dji.sampleV5.aircraft.data.DJIToastResult
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.key.KeyTools.createKey

import dji.sdk.keyvalue.value.camera.CameraMode
import dji.sdk.keyvalue.value.common.ComponentIndexType
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.error.RxError
import dji.v5.common.utils.CallbackUtils
import dji.v5.common.utils.RxUtil
import dji.v5.manager.KeyManager
import dji.v5.manager.datacenter.MediaDataCenter
import dji.v5.manager.datacenter.media.*
import dji.v5.utils.common.LogUtils
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.value.camera.CameraStorageLocation
import dji.sdk.keyvalue.value.common.EmptyMsg
import dji.sdk.keyvalue.value.file.FileListRequestTimeOrderType
import dji.v5.common.error.DJICommonError
import dji.v5.utils.common.ContextUtil
import dji.v5.utils.common.DiskUtil
import dji.v5.utils.common.StringUtils
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.util.ArrayList
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * @author feel.feng
 * @time 2022/04/20 2:19 下午
 * @description: 媒体回放下载数据
 */
class MediaVM : DJIViewModel() {
    var mediaFileListData = MutableLiveData<MediaFileListData>()
    var fileListState = MutableLiveData<MediaFileListState>()
    var isPlayBack = MutableLiveData<Boolean>()
    var componentIndex = MutableLiveData<ComponentIndexType>()

    // Guards init() so its listeners register only once per ViewModel lifetime; reset in
    // destroy() so re-entering a media screen re-registers them.
    private var initialized = false

    fun init() {
        if (initialized) return
        initialized = true
        addMediaFileListStateListener()
        mediaFileListData.value = MediaDataCenter.getInstance().mediaManager.mediaFileListData
        MediaDataCenter.getInstance().mediaManager.addMediaFileListStateListener { mediaFileListState ->
            if (mediaFileListState == MediaFileListState.UP_TO_DATE) {
                val data = MediaDataCenter.getInstance().mediaManager.mediaFileListData;
                mediaFileListData.postValue(data)
            }
        }

    }

    fun destroy() {
        initialized = false
        KeyManager.getInstance().cancelListen(this);
        removeAllFileListStateListener()

        MediaDataCenter.getInstance().mediaManager.release()
    }

    fun pullMediaFileListFromCamera(mediaFileIndex: Int, count: Int) {
        var currentTime = System.currentTimeMillis()
        MediaDataCenter.getInstance().mediaManager.pullMediaFileListFromCamera(
            PullMediaFileListParam.Builder().mediaFileIndex(mediaFileIndex).count(count).build(),
            object :
                CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    ToastUtils.showToast("Spend time:${(System.currentTimeMillis() - currentTime) / 1000}s")
                    LogUtils.i(logTag, "fetch success")
                }

                override fun onFailure(error: IDJIError) {
                    LogUtils.e(logTag, "fetch failed$error")
                }
            })
    }

    // Pull the file list and BLOCK until the camera signals the refresh is done — either the
    // MediaFileListState reaches UP_TO_DATE or the pull's own completion callback fires, whichever
    // comes first. Returns the fresh list straight from the media manager. Event-driven: the wait
    // ends on the SDK's completion signal, not a fixed delay. [timeoutMs] is only a safety cap so a
    // stalled link can't block the worker forever. Call from a worker thread.
    //
    // [count] caps how many files to fetch (-1 = the whole card); pair with [orderType] = NEW_FIRST
    // to fetch only the newest few, which is far cheaper than the whole list once the card is full.
    // Order/list contents never affect this app's correctness (callers filter by fileIndex), so the
    // defaults reproduce the original full-list pull.
    fun pullAndAwait(
        timeoutMs: Long,
        count: Int = -1,
        orderType: FileListRequestTimeOrderType? = null
    ): List<MediaFile> {
        val manager = MediaDataCenter.getInstance().mediaManager
        val latch = CountDownLatch(1)
        val stateListener = MediaFileListStateListener { state ->
            if (state == MediaFileListState.UP_TO_DATE) latch.countDown()
        }
        manager.addMediaFileListStateListener(stateListener)
        try {
            // Issue the pull on the main looper (matches how the rest of the app drives the SDK),
            // then block the calling worker on the latch.
            android.os.Handler(android.os.Looper.getMainLooper()).post {
                val builder = PullMediaFileListParam.Builder().mediaFileIndex(-1).count(count)
                if (orderType != null) builder.orderType(orderType)
                manager.pullMediaFileListFromCamera(
                    builder.build(),
                    object : CommonCallbacks.CompletionCallback {
                        override fun onSuccess() { latch.countDown() }
                        override fun onFailure(error: IDJIError) { latch.countDown() }
                    }
                )
            }
            latch.await(timeoutMs, TimeUnit.MILLISECONDS)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        } finally {
            manager.removeMediaFileListStateListener(stateListener)
        }
        return manager.mediaFileListData?.data ?: emptyList()
    }

    private fun addMediaFileListStateListener() {
        MediaDataCenter.getInstance().mediaManager.addMediaFileListStateListener(object :
            MediaFileListStateListener {
            override fun onUpdate(mediaFileListState: MediaFileListState) {
                fileListState.postValue(mediaFileListState)
            }

        })
    }

    private fun removeAllFileListStateListener() {
        MediaDataCenter.getInstance().mediaManager.removeAllMediaFileListStateListener()
    }

    fun getMediaFileList(): List<MediaFile> {
        return mediaFileListData.value?.data!!
    }

    fun setMediaFileXMPCustomInfo(info: String) {
        MediaDataCenter.getInstance().mediaManager.setMediaFileXMPCustomInfo(info, object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                toastResult?.postValue(DJIToastResult.success())
            }

            override fun onFailure(error: IDJIError) {
                toastResult?.postValue(DJIToastResult.failed(error.toString()))
            }
        })
    }

    fun getMediaFileXMPCustomInfo() {
        MediaDataCenter.getInstance().mediaManager.getMediaFileXMPCustomInfo(object :
            CommonCallbacks.CompletionCallbackWithParam<String> {
            override fun onSuccess(s: String) {
                toastResult?.postValue(DJIToastResult.success(s))
            }

            override fun onFailure(error: IDJIError) {
                toastResult?.postValue(DJIToastResult.failed(error.toString()))
            }
        })
    }

    fun setComponentIndex(index: ComponentIndexType) {
        componentIndex.postValue(index)
        isPlayBack.postValue(false)
        KeyManager.getInstance().cancelListen(this)
        KeyManager.getInstance().listen(
            createKey(
                CameraKey.KeyIsPlayingBack, index
            ), this
        ) { _, newValue ->
            newValue?.let {
                isPlayBack.postValue(it)
            }
        }
        val mediaSource = MediaFileListDataSource.Builder().setIndexType(index).build()
        MediaDataCenter.getInstance().mediaManager.setMediaFileDataSource(mediaSource)
    }

    fun setStorage(location: CameraStorageLocation) {
        val mediaSource = MediaFileListDataSource.Builder().setLocation(location).build()
        MediaDataCenter.getInstance().mediaManager.setMediaFileDataSource(mediaSource)
    }

    fun enable() {
        MediaDataCenter.getInstance().mediaManager.enable(object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                LogUtils.e(logTag, "enable playback success")
            }

            override fun onFailure(error: IDJIError) {
                LogUtils.e(logTag, "error is ${error.description()}")
            }
        })
    }

    fun disable() {
        MediaDataCenter.getInstance().mediaManager.disable(object :
            CommonCallbacks.CompletionCallback {
            override fun onSuccess() {
                LogUtils.e(logTag, "exit playback success")
            }

            override fun onFailure(error: IDJIError) {
                LogUtils.e(logTag, "error is ${error.description()}")
            }
        })
    }

    fun takePhoto(callback: CommonCallbacks.CompletionCallback) {
        val index = componentIndex.value
        if (index == null) {
            CallbackUtils.onFailure(callback, DJICommonError.FACTORY.build(DJICommonError.DISCONNECTED))
            return
        }
        val modeKey = createKey<CameraMode>(CameraKey.KeyCameraMode, index)
        val shoot = RxUtil.performActionWithOutResult(createKey(CameraKey.KeyStartShootPhoto, index))
        // Switching VIDEO->PHOTO on the camera costs a couple of seconds. We no longer restore
        // video after a capture, so the camera is usually already in PHOTO_NORMAL from a prior
        // shot — skip the redundant mode set in that case and shoot straight away.
        val capture =
            if (KeyManager.getInstance().getValue(modeKey) == CameraMode.PHOTO_NORMAL) shoot
            else RxUtil.setValue(modeKey, CameraMode.PHOTO_NORMAL).andThen(shoot)
        capture
            .subscribe({ CallbackUtils.onSuccess(callback) }
            ) { throwable: Throwable ->
                CallbackUtils.onFailure(
                    callback,
                    (throwable as RxError).djiError
                )
            }
    }

    // Restore the camera to video mode (takePhoto switches it to PHOTO_NORMAL), so the
    // live feed isn't left in photo mode after a capture.
    fun setVideoMode(callback: CommonCallbacks.CompletionCallback) {
        val index = componentIndex.value
        if (index == null) {
            CallbackUtils.onFailure(callback, DJICommonError.FACTORY.build(DJICommonError.DISCONNECTED))
            return
        }
        RxUtil.setValue(createKey<CameraMode>(CameraKey.KeyCameraMode, index), CameraMode.VIDEO_NORMAL)
            .subscribe({ CallbackUtils.onSuccess(callback) }
            ) { throwable: Throwable ->
                CallbackUtils.onFailure(
                    callback,
                    (throwable as RxError).djiError
                )
            }
    }

    fun formatSDCard(callback: CommonCallbacks.CompletionCallback) {
        val index = componentIndex.value
        if (index == null) {
            CallbackUtils.onFailure(callback, DJICommonError.FACTORY.build(DJICommonError.DISCONNECTED))
            return
        }
        KeyManager.getInstance().performAction(createKey(CameraKey.KeyFormatStorage, index), CameraStorageLocation.SDCARD, object :
            CommonCallbacks.CompletionCallbackWithParam<EmptyMsg> {
            override fun onSuccess(t: EmptyMsg?) {
                callback.onSuccess()
            }

            override fun onFailure(error: IDJIError) {
                callback.onFailure(error)
            }

        })
    }

    fun downloadMediaFile(mediaList: ArrayList<MediaFile>) {
        mediaList.forEach {
            downloadFile(it)
        }
    }

    private fun downloadFile(mediaFile: MediaFile) {
        val dirs = File(DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), "/mediafile"))
        if (!dirs.exists()) {
            dirs.mkdirs()
        }
        val filepath = DiskUtil.getExternalCacheDirPath(ContextUtil.getContext(), "/mediafile/" + mediaFile?.fileName)
        val file = File(filepath)
        var offset = 0L
        val outputStream = FileOutputStream(file, true)
        val bos = BufferedOutputStream(outputStream)
        mediaFile?.pullOriginalMediaFileFromCamera(offset, object : MediaFileDownloadListener {
            override fun onStart() {
                LogUtils.i("MediaFile", "${mediaFile.fileIndex} start download")
            }

            override fun onProgress(total: Long, current: Long) {
                val fullSize = offset + total;
                val downloadedSize = offset + current
                val data: Double = StringUtils.formatDouble((downloadedSize.toDouble() / fullSize.toDouble()))
                val result: String = StringUtils.formatDouble(data * 100, "#0").toString() + "%"
                LogUtils.i("MediaFile", "${mediaFile.fileIndex}  progress $result")
            }

            override fun onRealtimeDataUpdate(data: ByteArray, position: Long) {
                try {
                    bos.write(data)
                    bos.flush()
                } catch (e: IOException) {
                    LogUtils.e("MediaFile", "write error" + e.message)
                }
            }

            override fun onFinish() {
                try {
                    outputStream.close()
                    bos.close()
                } catch (error: IOException) {
                    LogUtils.e("MediaFile", "close error$error")
                }
                LogUtils.i("MediaFile", "${mediaFile.fileIndex}  download finish")
            }

            override fun onFailure(error: IDJIError?) {
                LogUtils.e("MediaFile", "download error$error")
            }

        })
    }
}