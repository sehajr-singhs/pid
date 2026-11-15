package org.firstinspires.ftc.robotcore.external;
/** STUB of the FTC SDK -- compile check only, not the real SDK. */
public interface Telemetry {
    Telemetry addData(String caption, String format, Object... args);
    Telemetry addData(String caption, Object value);
    void update();
}
