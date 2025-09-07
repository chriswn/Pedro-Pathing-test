# Full System Test (Vision + Drive + Scoring)

This directory includes `FullSystemTestOpMode` to validate AprilTag localization, tile driving, and the shooter.

## Prerequisites
- Hardware names:
  - Webcam: `Webcam 1`
  - Shooter motor (DcMotorEx): `shooter`
  - Optional launch angle servo: `launchAngle`
- Constants configured in `Constants/FieldConstants.java`.
- Shooter `SHOT_PROFILE` calibrated in `SubSystems/Scoring/ShooterSubsystem.java`.

## How to Run
1. Build and deploy the app to the robot.
2. From the Driver Station, select `Autonomous → FullSystemTestOpMode`.
3. Set alliance in code (`isBlueAlliance` boolean) if needed.
4. Place robot on field with AprilTags 20 (blue) and 24 (red) visible if possible.
5. Start the OpMode.

## What It Does
1. Initializes Drive, Vision, Shooter.
2. Localizes with `AprilTagNavigator.updateRobotPositionFromTriangulation()` (falls back to single-tag).
3. Drives to center of tile `C3`.
4. Calls `ShooterSubsystem.shootArtifact(currentPose, isBlueAlliance)` and holds for telemetry.

## Calibration Tips
- Update `SHOT_PROFILE` with measured `(distance, power, angle)` entries.
- Verify goal coordinates and field alignment in `FieldConstants` for your venue.
- Use telemetry from `AprilTagNavigator.updateDECODELocalizationTelemetry()` to monitor detections and pose.

## Troubleshooting
- No detections: check camera, lighting, tag placement, and confidence thresholds.
- Off-target shots: refine `SHOT_PROFILE` and verify `GOAL_HEIGHT_INCHES`.
- Drive overshoot: tune your drive power and consider adding PID/odometry.

See also:
- `TeamCode/src/main/java/README.md` for high-level module overview
- `SubSystems/Vision/APRILTAG_LOCALIZATION_README.md`
- `SubSystems/Drive/TILE_NAVIGATION_README.md`

