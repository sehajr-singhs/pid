# Characterising the FRC-equivalent mechanisms

1. Deploy `pidtool/runtime/frc/PidCalibrate.java` as a command.
2. It logs, via DataLogManager, to a .wpilog:
   - `sysid-test-state-<mechanism>` (string)
   - `pidtool/<mechanism>/Voltage`, `/Position`, `/Velocity`
3. Open the .wpilog in SysId, drag the entries into the Data Selector
   (docs.wpilib.org -> Software -> Advanced Controls -> System Identification
   -> Loading Data), then run the analysis for the mechanism type listed in
   `sysid_fields.json`.
4. SysId's kS/kV/kA replace the Piπd values in `Constants.java`; kS is
   *expected* to differ most, because Piπd models friction instead of
   measuring it.
