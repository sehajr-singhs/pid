# Characterising the FRC-equivalent mechanisms

1. Deploy `spool/runtime/frc/SpoolCalibrate.java` as a command.
2. It logs, via DataLogManager, to a .wpilog:
   - `sysid-test-state-<mechanism>` (string)
   - `spool/<mechanism>/Voltage`, `/Position`, `/Velocity`
3. Open the .wpilog in SysId, drag the entries into the Data Selector
   (docs.wpilib.org -> Software -> Advanced Controls -> System Identification
   -> Loading Data), then run the analysis for the mechanism type listed in
   `sysid_fields.json`.
4. SysId's kS/kV/kA replace the Spool values in `Constants.java`; kS is
   *expected* to differ most, because Spool models friction instead of
   measuring it.
