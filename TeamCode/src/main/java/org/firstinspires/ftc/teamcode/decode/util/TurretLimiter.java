package org.firstinspires.ftc.teamcode.decode.util;

/**
 * Safety helper for a turret driven purely by a motor-encoder position PID
 * (as Shooter.driveTurret() does): a soft tick-range limit and a stall
 * guard.
 *
 * Today nothing in Shooter/RobotConstants bounds turretTargetTicks. The
 * turret always turns to face a computed field angle, so if the robot's
 * pose estimate is ever wrong (a Pinpoint hiccup, a collision, auto ending
 * in an unexpected spot) the turret will happily command itself past
 * whatever the real mechanical range of motion is - most turrets on a
 * cable/wire run are limited to well under a full rotation. Driving a
 * stalled motor at full PID output for an extended period is how wires and
 * gearboxes get damaged.
 *
 * Usage (does not require editing Shooter.java, since turretTargetTicks and
 * driveTurret() are both public):
 * <pre>
 *   shooter.calculateTargets(goalPose);
 *   shooter.turretTargetTicks = limiter.clampTarget(shooter.turretTargetTicks); // optional, see below
 *   shooter.driveTurret();
 *   if (limiter.checkStall(shooter.turretMotor.getPower(), shooter.turretMotor.getCurrentPosition(), now)) {
 *       shooter.turretMotor.setPower(0.0);
 *       telemetry.addLine("TURRET STALL - power cut");
 *   }
 * </pre>
 *
 * clampTarget's min/max ticks are placeholders below - they must be
 * measured on the real robot (rotate the turret to each physical hard stop
 * and read the encoder) before this is relied on; a wrong guess would clamp
 * the turret short of a target it can actually reach. checkStall does NOT
 * need physical calibration - it only trips when the turret is being
 * commanded hard and genuinely isn't moving, so it's safe to enable as-is.
 */
public class TurretLimiter {

    private final int minTicks;
    private final int maxTicks;

    private final double stallPowerThreshold;
    private final int stallTickThreshold;
    private final double stallTimeSec;

    private double stalledSinceSec = -1.0;
    private int stallStartTicks = 0;

    public TurretLimiter(int minTicks, int maxTicks,
                          double stallPowerThreshold, int stallTickThreshold, double stallTimeSec) {
        this.minTicks = minTicks;
        this.maxTicks = maxTicks;
        this.stallPowerThreshold = stallPowerThreshold;
        this.stallTickThreshold = stallTickThreshold;
        this.stallTimeSec = stallTimeSec;
    }

    /** Clamp a requested absolute tick target to the turret's safe range of motion. */
    public int clampTarget(int requestedTicks) {
        return Math.max(minTicks, Math.min(maxTicks, requestedTicks));
    }

    /**
     * Call once per loop after driving the turret motor.
     *
     * @param commandedPower the power just sent to the turret motor.
     * @param currentTicks   the turret encoder position this loop.
     * @param nowSec         a monotonically increasing clock, e.g. an ElapsedTime.seconds().
     * @return true once the turret has been commanded hard for longer than
     *         stallTimeSec without moving more than stallTickThreshold -
     *         the caller should zero turret power and surface a fault.
     */
    public boolean checkStall(double commandedPower, int currentTicks, double nowSec) {
        boolean drivingHard = Math.abs(commandedPower) > stallPowerThreshold;

        if (!drivingHard) {
            stalledSinceSec = -1.0;
            return false;
        }

        if (stalledSinceSec < 0.0) {
            stalledSinceSec = nowSec;
            stallStartTicks = currentTicks;
            return false;
        }

        boolean barelyMoved = Math.abs(currentTicks - stallStartTicks) < stallTickThreshold;
        if (!barelyMoved) {
            stalledSinceSec = nowSec;
            stallStartTicks = currentTicks;
            return false;
        }

        return (nowSec - stalledSinceSec) >= stallTimeSec;
    }
}
