package dji.sampleV5.aircraft.controller

/**
 * Pilot / Safety authority arbitration for the HTTP command server.
 *
 * Two computers drive the drone over HTTP:
 *  - the Pilot Computer flies the mission normally;
 *  - the Safety Computer supervises and can seize control at any time.
 *
 * Authority is identified purely by an `X-Safety-Token` HTTP header (configured app-side):
 * a request carrying the valid token is [Source.SAFETY], everything else is [Source.PILOT].
 *
 * Behaviour:
 *  - Initial state: [Authority.PILOT] holds control; Pilot commands execute normally.
 *  - The FIRST control command from the Safety Computer latches control to [Authority.SAFETY].
 *    From then on every Pilot command is rejected.
 *  - The takeover is PERSISTENT: no timeout ever returns control to the Pilot. Even if the
 *    Safety Computer goes silent, the Pilot does not regain control.
 *  - The only way back is an explicit POST /releaseSafetyControl, callable solely by the
 *    Safety Computer, which returns authority to the Pilot.
 *
 * State is in-memory only (a @Volatile latch); an app restart resets to [Authority.PILOT],
 * which matches "restart == fresh mission". This object is orthogonal to DroneController's
 * RC manual-override latch — that tracks the physical RC pilot, this tracks which computer
 * commands the server.
 *
 * Thread-safety: the command server handles requests on a 10-thread pool, so the
 * check-and-latch in [authorizeControlCommand]/[releaseSafetyControl] is @Synchronized.
 */
object ControlAuthority {

    /** Which computer currently holds command authority. */
    enum class Authority { PILOT, SAFETY }

    /** Origin of an individual HTTP request, derived from the X-Safety-Token header. */
    enum class Source { PILOT, SAFETY }

    @Volatile
    var active: Authority = Authority.PILOT
        private set

    /** UI hook — fires only when [active] actually changes. */
    interface Listener {
        fun onAuthorityChanged(authority: Authority)
    }
    var listener: Listener? = null

    /**
     * Gate for every drone-control command (the /send/ family).
     * Returns true if the command is allowed to execute.
     *
     * A Safety command always passes and, on first arrival, latches authority to SAFETY
     * (cancelling any autonomous loop the Pilot left running). A Pilot command passes only
     * while the Pilot still holds authority.
     */
    @Synchronized
    fun authorizeControlCommand(source: Source): Boolean = when (source) {
        Source.SAFETY -> {
            if (active != Authority.SAFETY) {
                setAuthority(Authority.SAFETY)
                // Safety has seized control: stop whatever the Pilot was flying so the aircraft
                // holds position until the Safety Computer issues its own commands.
                DroneController.onSafetyTakeover()
            }
            true
        }
        Source.PILOT -> active == Authority.PILOT
    }

    /**
     * Explicit return of control to the Pilot. Reserved for the Safety Computer.
     * Returns false (and changes nothing) if a non-Safety caller invokes it.
     */
    @Synchronized
    fun releaseSafetyControl(source: Source): Boolean {
        if (source != Source.SAFETY) return false
        if (active != Authority.PILOT) setAuthority(Authority.PILOT)
        return true
    }

    private fun setAuthority(authority: Authority) {
        active = authority
        listener?.onAuthorityChanged(authority)
    }
}
