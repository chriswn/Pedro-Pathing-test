# Game-Specific Notes (DECODE)

Centralized notes for field/game constants and module usage. Source measurements: see `TeamCode/src/main/java/Competition Manual - V1.pdf`.

## Field and Goal Constants
- Defined in `Constants/FieldConstants.java`.
- Key values:
  - `FIELD_WIDTH_INCHES = 144.0`, `TILE_SIZE_INCHES = 24.0`
  - AprilTag size: `APRILTAG_SIZE_INCHES = 8.125`
  - Alliance goal tags (ID → x, y, heading°):
    - Blue goal (20): `(0.0, 72.0, 0.0)`
    - Red goal (24): `(144.0, 72.0, 180.0)`
  - Goal lip height: `GOAL_HEIGHT_INCHES = 38.75`

Adjust these if your venue’s field setup differs.

## Vision
- `SubSystems/Vision/AprilTagNavigator.java`
  - AprilTag-based localization.
  - `updateRobotPositionFromTriangulation()` fuses tags 20 & 24 when both visible; falls back to single tag.
  - Reads tag positions from `FieldConstants.APRILTAG_POSITIONS`.
- Tips: camera must be named `Webcam 1`. Use `updateDECODELocalizationTelemetry()` for DS telemetry.

## Scoring (Shooter)
- `SubSystems/Scoring/ShooterSubsystem.java`
  - Uses `FieldConstants.GOAL_X/Y_*` and `GOAL_HEIGHT_INCHES`.
  - `shootArtifact(TileCoordinate pos, boolean isBlue)` computes distance to goal, interpolates `SHOT_PROFILE` for motor power (and optional angle servo).
  - Replace placeholder `SHOT_PROFILE` entries with calibrated robot data.
- Calibration steps:
  1. Mark distances (20–60 in).
  2. At each distance, tune power/angle to score consistently.
  3. Record `(distance, power, angle)` into `SHOT_PROFILE`.
  4. Interpolation handles in-between distances.

## Drive
- `SubSystems/Drive/DriveSubsystem.java`
  - Maintains `TileCoordinate` pose and heading.
  - Works with Pedro Pathing or movement helpers (`moveToPosition`, `turnToTile`).
  - Vision updates pose through `AprilTagNavigator`.

## Typical Autonomous Flow
1. Init drive, vision, shooter.
2. Localize: `AprilTagNavigator.updateRobotPositionFromTriangulation()`.
3. Drive to chosen shot location.
4. Shoot: `shooter.shootArtifact(driveSubsystem.getCurrentPosition(), isBlueAlliance)`.
5. Iterate/refine; recalibrate `SHOT_PROFILE` as needed.

## Manual Pointers
- AprilTags: Section 9.10 (tag family, size, placement guidance).
- Tiles & coordinates: Section 9.4 (field and tile dimensions).
- Goal metrics: DECODE drawings; goal opening height 38.75 in.


