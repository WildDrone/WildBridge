package dji.sampleV5.aircraft

import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.et.create
import dji.v5.et.get

enum class DroneControlProfile(val displayName: String) {
    MINI_4_PRO("DJI Mini 4 Pro"),
    MATRICE_350_RTK("DJI Matrice 350 RTK")
}

object DroneControlProfiles {
    fun activeProfile(): DroneControlProfile {
        val detected = ProductKey.KeyProductType.create().get(ProductType.UNKNOWN)
        return fromProductType(detected)
    }

    fun fromProductType(productType: ProductType?): DroneControlProfile {
        val productName = productType?.name.orEmpty()
        return if (
            productName.contains("M350", ignoreCase = true) ||
            productName.contains("MATRICE_350", ignoreCase = true)
        ) {
            DroneControlProfile.MATRICE_350_RTK
        } else {
            DroneControlProfile.MINI_4_PRO
        }
    }

    fun isM350(): Boolean = activeProfile() == DroneControlProfile.MATRICE_350_RTK
}