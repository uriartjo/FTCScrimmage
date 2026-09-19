package org.firstinspires.ftc.teamcode.biobuzz.configs;

/**
 * Pedro Pathing v3 translation of the original PollenTrackConstants.java.
 * Moved from teamcode.decode.configs to teamcode.biobuzz.configs — this
 * season is "BioBuzz," not "Decode"; only the Pollen-tracking files live
 * here, everything else (RobotConstants, Shooter, the other teleops/autos)
 * is still under teamcode.decode.*.
 * All values unchanged. @Configurable (com.bylazar.configurables.annotations)
 * removed — same open dependency question as RobotConstants.java's javadoc:
 * com.bylazar:fullpanels isn't part of v3's dependency set and wasn't added
 * back to this project. Every field is still a plain re-readable static, so
 * PollenTrack still picks up edits on the next deploy — just not live from a
 * dashboard until that dependency question is resolved.
 */
public class PollenTrackConstants {

    // ─── Detection filtering ───────────────────────────────────────────────
    public static double CONFIDENCE_FLOOR = 45.0; // percent [0,100] — DetectorResults below this are ignored
    public static double AREA_TIE_BAND = 0.15;    // fraction — candidates within this of the max area are "roughly tied"
    public static int POLLEN_CLASS_ID = 0;        // yellow_pollen, per Hive-Vision's labels.txt class order

    // ─── Target lock persistence ───────────────────────────────────────────
    public static double MATCH_ANGLE_TOLERANCE_DEG = 6.0; // tx/ty delta under which two detections are "the same ball"
    public static int LOCK_SWITCH_FRAMES = 4;              // consecutive frames a challenger must win before the lock switches
    public static double LOST_TARGET_GRACE_SEC = 0.2;      // coast on the last command this long after losing the lock

    // ─── Arrival — TUNE IN FIELD: place a pollen ball at the desired pickup
    // distance, read the resulting lockedArea/lockedTx off telemetry, copy here ──
    public static double ARRIVED_AREA = 8.0;
    public static double ARRIVED_TX_TOLERANCE_DEG = 3.0;

    // ─── Steering PID (error = locked detection's tx in degrees, setpoint 0) ──
    public static double STEER_KP = 0.03;
    public static double STEER_KI = 0.0;
    public static double STEER_KD = 0.002;
    public static double STEER_I_MAX = 0.2;
    public static double STEER_OUT_MIN = -0.5;
    public static double STEER_OUT_MAX = 0.5;

    // ─── Forward power ramp ─────────────────────────────────────────────────
    public static double FORWARD_MAX_POWER = 0.45;
    public static double ALIGN_TX_DEG_FOR_FULL_FORWARD = 8.0;  // |tx| at/below this -> full forward ramp-in
    public static double ALIGN_TX_DEG_FOR_ZERO_FORWARD = 25.0; // |tx| at/above this -> zero forward
    public static double APPROACH_SLOWDOWN_START_AREA_FRACTION = 0.6; // fraction of ARRIVED_AREA where slowdown begins

    // ─── Limelight ──────────────────────────────────────────────────────────
    public static int LIMELIGHT_POLLEN_PIPELINE = 0; // pipeline index the Hive-Vision SSD-MobileNetV2 model is uploaded to
}
