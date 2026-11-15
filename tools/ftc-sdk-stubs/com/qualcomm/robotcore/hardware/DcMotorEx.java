package com.qualcomm.robotcore.hardware;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
/** STUB of the FTC SDK -- compile check only, not the real SDK. */
public interface DcMotorEx extends DcMotor {
    double getVelocity();
    double getCurrent(CurrentUnit unit);
}
