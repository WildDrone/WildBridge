package dji.v5.ux.core.widget.fpv

import dji.sdk.keyvalue.value.common.CameraLensType
import dji.sdk.keyvalue.value.common.ComponentIndexType

/**
 * Class Description
 *
 * @author Hoker
 * @date 2022/3/19
 *
 * Copyright (c) 2022, DJI All Rights Reserved.
 */
interface FPVStreamSourceListener {
    fun onStreamSourceUpdated(devicePosition: ComponentIndexType, lensType: CameraLensType)
}