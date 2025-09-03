package dji.sampleV5.aircraft.pages

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.activityViewModels
import dji.sampleV5.aircraft.databinding.FragTtsBinding
import dji.sampleV5.aircraft.models.MegaphoneVM
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.manager.aircraft.megaphone.FileInfo
import dji.v5.manager.aircraft.megaphone.PlayMode
import dji.v5.manager.aircraft.megaphone.UploadType
import dji.v5.utils.common.LogPath
import dji.v5.utils.common.LogUtils

/**
 * Description : TTS fragment
 * Author : daniel.chen
 * CreateDate : 2022/1/17 2:41 下午
 * Copyright : ©2021 DJI All Rights Reserved.
 */
class TTSFragment: DJIFragment(){
    private val megaphoneVM: MegaphoneVM by activityViewModels()
    private var binding: FragTtsBinding? = null

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        binding = FragTtsBinding.inflate(inflater, container, false)
        return binding?.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        initBtnListener()
    }

    private fun initBtnListener() {
        binding?.btnTts?.setOnClickListener {
            binding?.etTts?.text.let {
                var fileInfo = FileInfo(
                    UploadType.TTS_DATA,
                    null,
                    binding?.etTts?.text.toString().toByteArray(Charsets.UTF_8)
                )
                megaphoneVM.pushFileToMegaphone(fileInfo,
                    object : CommonCallbacks.CompletionCallbackWithProgress<Int> {
                        override fun onProgressUpdate(progress: Int) {
                            binding?.tvTtsUploadStatus?.text = "upload Progress：$progress"
                        }

                        override fun onSuccess() {
                            binding?.tvTtsUploadStatus?.text = "upload success!"
                            megaphoneVM.startPlay(object:CommonCallbacks.CompletionCallback{
                                override fun onSuccess() {
                                    LogUtils.i("TTS","Start Play Success")
                                }

                                override fun onFailure(error: IDJIError) {
                                    LogUtils.i("TTS","Start Play Failed")
                                }
                            })
                        }

                        override fun onFailure(error: IDJIError) {
                            binding?.tvTtsUploadStatus?.text = "upload failed: $error"
                        }
                    })
            }
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        megaphoneVM.getPlayMode(object :CommonCallbacks.CompletionCallbackWithParam<PlayMode>{
            override fun onSuccess(t: PlayMode?) {
               if (t != PlayMode.UNKNOWN) {
                   megaphoneVM.stopPlay(null)
               }
            }

            override fun onFailure(error: IDJIError) {
               LogUtils.e(LogPath.SAMPLE , "Get Play Mode Failed!")
            }

        })

    }
}