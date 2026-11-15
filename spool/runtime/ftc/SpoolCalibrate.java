package org.firstinspires.ftc.teamcode.spool;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;

/**
 * SpoolCalibrate: one short logged test run to finish the job the CAD started.
 *
 * <p>Applies three blocks and logs voltage, velocity, position and current to a
 * CSV that {@code spool fit} turns into real kS / kV / kA:
 *
 * <ol>
 *   <li>slow ramp 0 -> 2.0 V in 0.1 V steps: the speed where it first moves is kS
 *   <li>steady 2, 3, 4, 5, 6, 7 V: slope of speed vs voltage is kV
 *   <li>steps 3, 6, 9, 6, 3 V: the transient gives kA
 * </ol>
 *
 * <p>Only the FTC SDK is used: no new dependencies, no external libraries.
 *
 * <p>Before running: set MOTOR_NAME to the name in the robot configuration, set
 * TICKS_PER_REV for that motor (28 for a goBILDA 5203 at the output shaft, 384.5
 * for the 435 rpm gearmotor, 537.7 for 312 rpm), and put the mechanism in a state
 * where it can spin freely.
 *
 * <p>The log lands at /sdcard/FIRST/spool/&lt;MOTOR_NAME&gt;.csv.
 */
@TeleOp(name = "SpoolCalibrate", group = "Spool")
public class SpoolCalibrate extends LinearOpMode {

    // ------------------------------------------------------------------ config
    private static final String MOTOR_NAME = "flywheel";
    private static final double TICKS_PER_REV = 28.0;   // encoder counts per output rev
    private static final double RAMP_STEP_VOLTS = 0.1;
    private static final double RAMP_STEP_SECONDS = 0.25;
    private static final double RAMP_MAX_VOLTS = 2.0;
    private static final double[] STEADY_VOLTS = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
    private static final double STEADY_SECONDS = 0.5;
    private static final double[] STEP_VOLTS = {3.0, 6.0, 9.0, 6.0, 3.0};
    private static final double STEP_SECONDS = 0.3;
    private static final double LOOP_SECONDS = 0.01;    // 100 Hz logging

    private DcMotorEx motor;
    private VoltageSensor battery;
    private PrintWriter log;
    private double commandedVolts = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        battery = hardwareMap.voltageSensor.iterator().next();

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Spool", "spooling... motor=" + MOTOR_NAME);
        telemetry.addData("Spool", "clear the mechanism, then press start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        try {
            openLog();
            writeRow(0.0);
            ramp();
            steady();
            steps();
        } finally {
            commandedVolts = 0.0;
            motor.setPower(0.0);
            if (log != null) {
                log.flush();
                log.close();
            }
        }

        telemetry.addData("Spool", "done: /sdcard/FIRST/spool/" + MOTOR_NAME + ".csv");
        telemetry.update();
        sleep(2000);
    }

    private void ramp() {
        int n = (int) Math.round(RAMP_MAX_VOLTS / RAMP_STEP_VOLTS);
        for (int i = 1; i <= n && opModeIsActive(); i++) {
            applyVolts(i * RAMP_STEP_VOLTS, RAMP_STEP_SECONDS);
        }
    }

    private void steady() {
        for (double v : STEADY_VOLTS) {
            if (!opModeIsActive()) {
                return;
            }
            applyVolts(v, STEADY_SECONDS);
        }
    }

    private void steps() {
        for (double v : STEP_VOLTS) {
            if (!opModeIsActive()) {
                return;
            }
            applyVolts(v, STEP_SECONDS);
        }
    }

    /**
     * Hold a voltage for `seconds`, logging at LOOP_SECONDS.
     *
     * <p>setPower is a fraction of battery voltage, so the fraction is recomputed
     * every loop from the measured pack voltage. That keeps the logged voltage
     * equal to the voltage actually applied, which is what the fit assumes.
     */
    private void applyVolts(double volts, double seconds) {
        commandedVolts = volts;
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < seconds && opModeIsActive()) {
            double pack = Math.max(battery.getVoltage(), 1.0);
            motor.setPower(volts / pack);
            writeRow(timer.seconds());
            telemetry.addData("t", "%.2f", timer.seconds());
            telemetry.addData("volts", "%.2f", volts);
            telemetry.addData("rpm", "%.1f", motor.getVelocity() / TICKS_PER_REV * 60.0);
            telemetry.update();
            sleep((long) (LOOP_SECONDS * 1000));
        }
    }

    private void openLog() {
        File dir = new File("/sdcard/FIRST/spool");
        if (!dir.exists() && !dir.mkdirs()) {
            telemetry.addData("Spool", "could not create " + dir);
            telemetry.update();
        }
        File file = new File(dir, MOTOR_NAME + ".csv");
        try {
            log = new PrintWriter(new FileWriter(file, false));
            // t,voltage,velocity,position,current   (SI: s, V, rad/s, rad, A)
            log.println("t,voltage,velocity,position,current");
        } catch (IOException e) {
            telemetry.addData("Spool", "log open failed: " + e.getMessage());
            telemetry.update();
            log = null;
        }
    }

    private void writeRow(double elapsedSeconds) {
        if (log == null) {
            return;
        }
        double velocityRadPerSec = motor.getVelocity() / TICKS_PER_REV * 2.0 * Math.PI;
        double positionRad = motor.getCurrentPosition() / TICKS_PER_REV * 2.0 * Math.PI;
        double currentAmps = readCurrent();
        log.println(String.format(
                Locale.US, "%.4f,%.4f,%.5f,%.5f,%.4f",
                elapsedSeconds, commandedVolts, velocityRadPerSec, positionRad, currentAmps));
    }

    /**
     * Current draw. getCurrent(CurrentUnit.AMPS) exists on DcMotorEx, but the
     * goBILDA 5203 does not report current, so a failure here must not stop the
     * run: the fit does not need the current column.
     */
    private double readCurrent() {
        try {
            return motor.getCurrent(CurrentUnit.AMPS);
        } catch (RuntimeException e) {
            return 0.0;
        }
    }
}
