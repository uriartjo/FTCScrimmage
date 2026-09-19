package org.firstinspires.ftc.teamcode.decode.util;

/**
 * Generic PID controller with clamped output/integral and an optional
 * low-pass filter on the derivative term.
 *
 * The codebase currently hand-rolls this same P/I/D math independently in
 * four places: Shooter.driveTurret(), Shooter.driveFlywheels(),
 * Blue_18Ball.updateShooterVelocityPID() (and its five auto siblings), and
 * ShooterTest.updateFlywheelVelocityPID(). All four use raw
 * (error - lastError) / dt for the D term, which amplifies encoder-tick
 * quantization noise into large power spikes whenever dt is small (the
 * turret loop clamps dt down to 0.0005s, i.e. divides by up to 2000).
 * Every copy also has to independently get the "clamp the dt" and
 * "clamp the I-term, not the raw integral" details right.
 *
 * This class centralizes that logic in one place and adds an optional
 * derivative filter (see {@code derivativeFilterAlpha}) so a future tuning
 * fix or noise fix only needs to happen once. It's a plain utility with no
 * FTC SDK dependency, so it's easy to unit test in isolation.
 *
 * Gains are mutable via {@link #setGains} specifically so a caller reading
 * live-tunable constants (e.g. an {@code @Configurable} class read from a
 * dashboard) can push updated gains into an already-running controller
 * instance without losing its integral/derivative history — that history is
 * the reason to keep one persistent instance in the first place, so
 * "re-tune" must not mean "reconstruct and lose it."
 */
public class PIDController {

    private static final double MIN_DT_SEC = 0.0005;
    private static final double MAX_DT_SEC = 0.05;

    private double kP, kI, kD;
    private final double integralMax;
    private final double outputMin, outputMax;

    /**
     * 0.0 = no filtering (matches existing behavior exactly).
     * Closer to 1.0 = heavier smoothing of the derivative term, at the cost
     * of some added phase lag. 0.2-0.4 is a reasonable starting point if
     * the D term is found to be spiking on real encoder noise.
     */
    private final double derivativeFilterAlpha;

    private double integral = 0.0;
    private double lastError = 0.0;
    private double filteredDerivative = 0.0;
    private boolean hasLastError = false;

    public PIDController(double kP, double kI, double kD,
                          double integralMax, double outputMin, double outputMax) {
        this(kP, kI, kD, integralMax, outputMin, outputMax, 0.0);
    }

    public PIDController(double kP, double kI, double kD,
                          double integralMax, double outputMin, double outputMax,
                          double derivativeFilterAlpha) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralMax = integralMax;
        this.outputMin = outputMin;
        this.outputMax = outputMax;
        this.derivativeFilterAlpha = derivativeFilterAlpha;
    }

    /**
     * @param error current setpoint - measured value, in whatever units the
     *              caller is controlling (ticks, TPS, degrees, ...).
     * @param dtSecRaw elapsed time since the previous update, in seconds.
     * @return clamped controller output in [outputMin, outputMax].
     */
    public double update(double error, double dtSecRaw) {
        double dtSec = clamp(dtSecRaw, MIN_DT_SEC, MAX_DT_SEC);

        double pTerm = kP * error;

        integral += error * dtSec;
        double iTerm = clamp(kI * integral, -integralMax, integralMax);

        double rawDerivative = hasLastError ? (error - lastError) / dtSec : 0.0;
        filteredDerivative = derivativeFilterAlpha <= 0.0
                ? rawDerivative
                : derivativeFilterAlpha * filteredDerivative + (1.0 - derivativeFilterAlpha) * rawDerivative;
        double dTerm = kD * filteredDerivative;

        lastError = error;
        hasLastError = true;

        return clamp(pTerm + iTerm + dTerm, outputMin, outputMax);
    }

    /**
     * Updates the gains in place, without touching integral/derivative
     * history — safe to call every loop with freshly-read live-tunable
     * constants. Only integralMax/outputMin/outputMax/derivativeFilterAlpha
     * stay fixed for the life of the instance; those are structural output
     * bounds rather than tuning knobs a driver would adjust mid-session.
     */
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /** Zeroes the integral and derivative history. Call when re-engaging after a hold/disable. */
    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        filteredDerivative = 0.0;
        hasLastError = false;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
