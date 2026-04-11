# Pinpoint Removal Migration Gates

These tests are the required pre/post gate for high-risk pose-estimation changes.
Run them before changing pose authority, frame transforms, heading seeding, or
vision timestamp wiring, then run the same suite again after each such change.

## Required suite

- `frc.robot.Commands.ScoreInHubTest`
  - Driver-frame to field-frame velocity transforms
  - Field-heading to driver-heading conversion
  - Existing score-in-hub viability/latch invariants
- `frc.robot.subsystems.PoseEstimatorMigrationGateTest`
  - Fresh-estimator invalid/null baseline
  - Immediate fused-pose and prediction availability after reset
  - Stationary positive-lookahead contract after reset
  - Field-relative predicted velocity contract across nonzero robot heading
- `frc.robot.LimelightHelpersMigrationGateTest`
  - Limelight published timestamp minus latency conversion baseline
- `frc.robot.LimelightSubsystemMigrationGateTest`
  - Direct pass-through of official Limelight pose-estimate timestamps into the CTRE vision path

## Baseline command

```powershell
$env:JAVA_HOME='C:\Users\Public\wpilib\2026\jdk'
$env:PATH='C:\Users\Public\wpilib\2026\jdk\bin;' + $env:PATH
$env:GRADLE_USER_HOME="$PWD\.gradle-user-home"
.\gradlew.bat test `
  --tests frc.robot.Commands.ScoreInHubTest `
  --tests frc.robot.subsystems.PoseEstimatorMigrationGateTest `
  --tests frc.robot.LimelightHelpersMigrationGateTest `
  --tests frc.robot.LimelightSubsystemMigrationGateTest
```

## Recorded baseline

- Date: `2026-04-10`
- Result: `PASS`
- Test counts:
  - `frc.robot.Commands.ScoreInHubTest`: `13`
  - `frc.robot.subsystems.PoseEstimatorMigrationGateTest`: `4`
  - `frc.robot.LimelightHelpersMigrationGateTest`: `1`
  - Total: `18`

## Step 3 addition

- Date: `2026-04-10`
- Added `frc.robot.LimelightSubsystemMigrationGateTest` to lock direct use of the official Limelight pose-estimate timestamp path for CTRE vision injection.
