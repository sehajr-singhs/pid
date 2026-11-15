package com.qualcomm.robotcore.hardware;
/** STUB of the FTC SDK -- compile check only, not the real SDK. */
public interface DcMotor {
    enum RunMode { RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, STOP_AND_RESET_ENCODER }
    enum ZeroPowerBehavior { FLOAT, BRAKE, UNKNOWN }
    void setMode(RunMode mode);
    void setZeroPowerBehavior(ZeroPowerBehavior behavior);
    void setPower(double power);
    int getCurrentPosition();
}
