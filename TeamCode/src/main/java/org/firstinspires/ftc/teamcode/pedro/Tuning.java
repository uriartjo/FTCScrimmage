package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

/**
 * Registers this robot's tuning procedures with Pedro v3's autotune web
 * dashboard (started automatically on the Robot/Control Hub — see
 * TuningSession/WebServer in com.pedropathing:tuning — no OpMode needed).
 *
 * Without the @Tuner-annotated methods below, TunerScanner finds nothing:
 * MecanumTuner/PinpointTuner/ForesightTuner existed in procedures/ but had
 * no registration point, so the dashboard's tuner list was empty and there
 * was no way to actually run them on the robot. This file was the empty
 * stub ("// Tuners go here") where that registration belongs.
 */
public class Tuning {

    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // Foresight needs a working Localizer + Drivetrain to drive/measure with —
    // reuses the same Mecanum/Pinpoint wiring already set up in Constants.java.
    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(
            h -> new PinpointLocalizer(h, Constants.pinpointConfig),
            h -> new Mecanum(h, Constants.mecanumConfig)
        );
    }
}
