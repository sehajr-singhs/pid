package frc.robot.spool;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SpoolCalibrate (FRC): the same three-block test as the FTC OpMode, written as
 * a WPILib command so it can be bound to a button.
 *
 * <p>Logs through DataLogManager so the resulting .wpilog can be opened directly
 * in SysId (docs.wpilib.org, System Identification -> Loading Data). The entry
 * names are the ones SysId looks for:
 *
 * <pre>
 *   sysid-test-state-&lt;mechanism&gt;   (string)
 *   spool/&lt;mechanism&gt;/Voltage      (double, V)
 *   spool/&lt;mechanism&gt;/Position     (double)
 *   spool/&lt;mechanism&gt;/Velocity     (double)
 * </pre>
 *
 * <p>Motor control is through the WPILib MotorController interface, so this
 * works with any vendor controller that implements it. setVoltage() is used
 * rather than set() so battery sag does not change the test voltage.
 *
 * <p>Not compiled in this session: the WPILib vendordep is not installed on this
 * laptop. The FTC OpMode is the one that gets a compile check.
 */
public class SpoolCalibrate extends SubsystemBase {

    private static final double RAMP_STEP_VOLTS = 0.1;
    private static final double RAMP_STEP_SECONDS = 0.25;
    private static final double RAMP_MAX_VOLTS = 2.0;
    private static final double[] STEADY_VOLTS = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    private static final double STEADY_SECONDS = 0.5;
    private static final double[] STEP_VOLTS = {3.0, 6.0, 9.0, 6.0, 3.0};
    private static final double STEP_SECONDS = 0.3;
    private static final double LOOP_SECONDS = 0.01;

    private final String mechanism;
    private final MotorController motor;
    private final Encoder encoder;
    private final double distancePerPulse;

    private final PIDController holdPid = new PIDController(0.0, 0.0, 0.0);
    private boolean holding = false;

    /**
     * @param mechanism short name used in log entries, e.g. "flywheel"
     * @param motor     the motor controller
     * @param encoder   encoder on the same shaft
     * @param countsPerRevolution encoder counts per output revolution
     */
    public SpoolCalibrate(String mechanism, MotorController motor, Encoder encoder,
                          double countsPerRevolution) {
        this.mechanism = mechanism;
        this.motor = motor;
        this.encoder = encoder;
        // Position in revolutions, velocity in rev/s; SysId converts with these.
        this.distancePerPulse = 1.0 / countsPerRevolution;
        encoder.setDistancePerPulse(distancePerPulse);
        encoder.reset();
    }

    /** The whole test as one command. */
    public Command characterizeCommand() {
        return run(() -> {
            DataLogManager.log("sysid-test-state-" + mechanism);
            ramp();
            steady();
            steps();
        })
                .beforeStarting(() -> {
                    holding = false;
                    motor.set(0.0);
                })
                .finallyDo(interrupted -> {
                    motor.set(0.0);
                    DataLogManager.log("sysid-test-state-" + mechanism + "-done");
                })
                .withName("SpoolCalibrate[" + mechanism + "]");
    }

    /** Hold a voltage for a fixed time, logging every loop. */
    private void applyVolts(double volts, double seconds) {
        double deadline = Timer.getFPGATimestamp() + seconds;
        while (Timer.getFPGATimestamp() < deadline) {
            motor.setVoltage(volts);
            logSample(volts);
            Timer.delay(LOOP_SECONDS);
            if (Thread.currentThread().isInterrupted()) {
                return;
            }
        }
    }

    private void logSample(double volts) {
        DataLogManager.log("spool/" + mechanism + "/Voltage", "" + volts);
        DataLogManager.log("spool/" + mechanism + "/Position",
                "" + encoder.getDistance());
        DataLogManager.log("spool/" + mechanism + "/Velocity",
                "" + encoder.getRate());
    }

    private void ramp() {
        int n = (int) Math.round(RAMP_MAX_VOLTS / RAMP_STEP_VOLTS);
        for (int i = 1; i <= n; i++) {
            applyVolts(i * RAMP_STEP_VOLTS, RAMP_STEP_SECONDS);
        }
    }

    private void steady() {
        for (double v : STEADY_VOLTS) {
            applyVolts(v, STEADY_SECONDS);
        }
    }

    private void steps() {
        for (double v : STEP_VOLTS) {
            applyVolts(v, STEP_SECONDS);
        }
    }

    /** Optional: hold a position with the gains Spool exported. */
    public void holdAt(double positionRad) {
        holdPid.setSetpoint(positionRad);
        holding = true;
    }

    @Override
    public void periodic() {
        if (!holding) {
            return;
        }
        double position = encoder.getDistance() * 2.0 * Math.PI;
        motor.setVoltage(holdPid.calculate(position));
    }
}
