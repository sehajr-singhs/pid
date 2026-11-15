package com.qualcomm.robotcore.eventloop.opmode;
import java.lang.annotation.*;
/** STUB of the FTC SDK annotation -- compile check only, not the real SDK. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface TeleOp { String name() default ""; String group() default ""; }
