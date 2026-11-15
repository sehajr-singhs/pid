package com.qualcomm.robotcore.eventloop.opmode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
/** STUB of the FTC SDK -- compile check only, not the real SDK. */
public abstract class LinearOpMode {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public abstract void runOpMode() throws InterruptedException;
    public void waitForStart() {}
    public boolean opModeIsActive() { return true; }
    public boolean isStopRequested() { return false; }
    public void sleep(long ms) {}
}
