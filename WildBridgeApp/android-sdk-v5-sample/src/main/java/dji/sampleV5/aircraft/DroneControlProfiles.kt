package dji.sampleV5.aircraft

import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.manager.aircraft.payload.PayloadIndexType

enum class DroneControlProfile(
    val displayName: String,
    val maxHorizontalSpeedMps: Double,
    val maxHorizontalAccelMps2: Double,
    val distanceKp: Double,
    // Integral gain for the distance controller. Held at 0: the distance error is
    // sign-definite (always >= 0), so an integral term can only wind up and never
    // unwind, dumping into the output near the waypoint and causing overshoot.
    val distanceKi: Double,
    val distanceKd: Double,
    val yawKp: Double,
    val maxYawRateDegS: Double,
    val defaultCruiseSpeedMps: Double,
    // Payload SDK index (gimbal position / SkyPort) the release/drop payload is wired to,
    // or null if this drone carries no droppable payload. Consumed by the /send/drop
    // endpoint. e.g. RIGHT for the M300 SkyPort release payload (TH4), PORT_3 elsewhere.
    val payloadIndexType: PayloadIndexType?,
    // Payload widget indices /send/drop pulses: the unlock SWITCH then the release BUTTON.
    // Default SWITCH 0 / BUTTON 1 is the original working PORT_3 logic (M400/M3E/M350); the
    // M300 SkyPort payload (TH4) overrides to its config-interface Unlock 3 / All_Down 5.
    val dropArmSwitchIndex: Int = 0,
    val dropReleaseButtonIndex: Int = 1
) {
    MAVIC_3_ENTERPRISE(
        displayName = "Mavic 3 Enterprise",
        maxHorizontalSpeedMps = 20.0,
        maxHorizontalAccelMps2 = 1.0,
        distanceKp = 0.65,
        distanceKi = 0.0,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 15.0,
        payloadIndexType = null
    ),
    MATRICE_300_RTK(
        displayName = "Matrice 300 RTK",
        maxHorizontalSpeedMps = 20.0,
        maxHorizontalAccelMps2 = 1.0,
        distanceKp = 0.35,
        distanceKi = 0.0,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 25.0,
        // SkyPort release payload (TH4) sits on the RIGHT gimbal position; its drop is the
        // config-interface Unlock SWITCH (3) + All_Down BUTTON (5).
        payloadIndexType = PayloadIndexType.RIGHT,
        dropArmSwitchIndex = 3,
        dropReleaseButtonIndex = 5
    ),
    MATRICE_350_RTK(
        displayName = "Matrice 350 RTK",
        maxHorizontalSpeedMps = 20.0,
        maxHorizontalAccelMps2 = 1.0,
        distanceKp = 0.35,
        distanceKi = 0.0,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 3.0,
        // Same SkyPort release payload as the M300: RIGHT + Unlock 3 / All_Down 5.
        payloadIndexType = PayloadIndexType.RIGHT,
        dropArmSwitchIndex = 3,
        dropReleaseButtonIndex = 5
    ),
    MATRICE_400(
        displayName = "Matrice 400",
        maxHorizontalSpeedMps = 25.0,
        maxHorizontalAccelMps2 = 1.0,
        distanceKp = 0.35,
        distanceKi = 0.0,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 25.0,
        payloadIndexType = PayloadIndexType.PORT_4,
        dropArmSwitchIndex = 3,
        dropReleaseButtonIndex = 5
    ),
    MINI_4_PRO(
        displayName = "DJI Mini 4 Pro",
        maxHorizontalSpeedMps = 15.0,
        maxHorizontalAccelMps2 = 1.0,
        distanceKp = 0.65,
        distanceKi = 0.0,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 2.0,
        payloadIndexType = null
    )
}

object DroneControlProfiles {
    fun activeProfile(): DroneControlProfile {
        val detected = ProductKey.KeyProductType.create().get(ProductType.UNKNOWN)
        return fromProductType(detected)
    }

    fun fromProductType(productType: ProductType?): DroneControlProfile {
        val name = productType?.name.orEmpty()
        return when {
            name.contains("MATRICE_400", ignoreCase = true) ||
            name.contains("M400", ignoreCase = true) -> DroneControlProfile.MATRICE_400

            name.contains("M350", ignoreCase = true) ||
            name.contains("MATRICE_350", ignoreCase = true) -> DroneControlProfile.MATRICE_350_RTK

            name.contains("M300", ignoreCase = true) ||
            name.contains("MATRICE_300", ignoreCase = true) -> DroneControlProfile.MATRICE_300_RTK

            name.contains("MINI_4", ignoreCase = true) ||
            name.contains("MINI4", ignoreCase = true) -> DroneControlProfile.MINI_4_PRO

            name.contains("MAVIC_3", ignoreCase = true) ||
            name.contains("MAVIC3", ignoreCase = true) ||
            name.contains("M3E", ignoreCase = true) ||
            name.contains("WM265", ignoreCase = true) -> DroneControlProfile.MAVIC_3_ENTERPRISE

            else -> DroneControlProfile.MAVIC_3_ENTERPRISE
        }
    }
}