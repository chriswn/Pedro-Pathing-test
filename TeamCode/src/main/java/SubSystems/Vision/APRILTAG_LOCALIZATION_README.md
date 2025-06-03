# AprilTag-Based Tile Localization System for DECODE Field

This system uses AprilTag detection to determine the robot's current tile position on the FTC DECODE field, enabling precise tile-based navigation.

**Based on FTC Competition Manual Section 9.10 AprilTags**

## Overview

The `AprilTagNavigator` class has been completely redesigned to focus on **localization** rather than alignment. It uses AprilTag detections to calculate the robot's field position and determine which tile the robot is currently on.

## Key Features

✅ **Tile-Based Localization**: Determines robot's current tile position using AprilTag detection
✅ **Field Position Calculation**: Converts AprilTag relative positions to absolute field coordinates
✅ **Integration with DriveSubsystem**: Automatically updates robot's tile position and heading
✅ **Robust Detection**: Filters detections by distance and confidence for reliability
✅ **Comprehensive Telemetry**: Detailed localization information and debugging data

## How It Works

1. **AprilTag Detection**: Detects AprilTags in the camera view
2. **Field Position Lookup**: Looks up the known field position of detected AprilTags
3. **Relative Position Calculation**: Uses the robot's relative position to the AprilTag
4. **Coordinate Transformation**: Converts relative position to absolute field coordinates
5. **Tile Position Update**: Updates the DriveSubsystem with current tile position

## DECODE Field AprilTag Configuration

### AprilTag Specifications
- **Size**: 8.125 inches (~20.65 cm) square
- **Tag Family**: 36h11
- **Reliable Tags for Navigation**: 
  - **Blue Alliance Goal**: Tag ID 20
  - **Red Alliance Goal**: Tag ID 24
- **NOT Recommended**: Obelisk tags (IDs 21, 22, 23) - placement varies between matches

### Field Positions (Pre-configured)
The system is pre-configured with the correct DECODE field positions:

```java
private static final double[][] APRILTAG_POSITIONS = {
    { 20, 0, 72, 0 },    // Blue Alliance Goal - left edge, center height, facing right
    { 24, 144, 72, 180 }, // Red Alliance Goal - right edge, center height, facing left
};
```

**Format**: `{tagId, x_inches, y_inches, heading_degrees}`

### 2. Camera Configuration

Ensure your camera is properly configured in the hardware map:

```java
// In your opmode
aprilTagNavigator = new AprilTagNavigator(drive, hardwareMap, telemetry);
```

## Usage Examples

### Basic DECODE Localization

```java
// Initialize the system
DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
AprilTagNavigator aprilTagNavigator = new AprilTagNavigator(drive, hardwareMap, telemetry);

// Update robot position using alliance goal detection (recommended for DECODE)
boolean localized = aprilTagNavigator.updateRobotPositionFromAllianceGoals();

if (localized) {
    TileCoordinate currentPos = drive.getCurrentPosition();
    String tilePosition = currentPos.getTilePosition(); // "C3"
    String tabPosition = currentPos.getTabPosition();   // "X3"
    
    // Check which alliance goal was detected
    var bestDetection = aprilTagNavigator.getBestAllianceGoalDetection();
    String alliance = aprilTagNavigator.getAllianceColor(bestDetection); // "Blue" or "Red"
}
```

### Continuous DECODE Localization

```java
// In your main loop
java
public class ExampleAprilTagLocalizationOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        AprilTagNavigator aprilTagNavigator = new AprilTagNavigator(drive, hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            boolean localized = aprilTagNavigator.updateRobotPositionFromAllianceGoals();
            if (localized) {
                TileCoordinate currentPos = drive.getCurrentPosition();
                AprilTagDetection bestDetection = aprilTagNavigator.getBestAllianceGoalDetection();
                if (aprilTagNavigator.isBlueAllianceGoal(bestDetection)) {
                    drive.moveToTile('C', 3, 12, 12, 0.5);
                } else if (aprilTagNavigator.isRedAllianceGoal(bestDetection)) {
                    drive.moveToTile('D', 3, 12, 12, 0.5);
                }
            }
            aprilTagNavigator.updateDECODELocalizationTelemetry();
            telemetry.update();
        }
    }
}
```

### Detection Quality Monitoring

```java
// Check if robot is currently localized
boolean isLocalized = aprilTagNavigator.isLocalized();

// Get all valid detections
List<AprilTagDetection> detections = aprilTagNavigator.getAllDetections();

// Get best detection
AprilTagDetection bestDetection = aprilTagNavigator.getBestDetection();

// Get detailed localization info
String info = aprilTagNavigator.getLocalizationInfo();
```

## API Reference

### Core Methods

#### `updateRobotPosition()`
Updates the robot's position using AprilTag detection.
- **Returns**: `boolean` - True if position was successfully updated
- **Side Effects**: Updates DriveSubsystem position and heading

#### `getCurrentTilePosition()`
Gets the robot's current tile position.
- **Returns**: `TileCoordinate` - Current position or null if not localized

#### `isLocalized()`
Checks if robot is currently localized.
- **Returns**: `boolean` - True if localized using AprilTags

### Detection Methods

#### `getBestDetection()`
Gets the best AprilTag detection for localization.
- **Returns**: `AprilTagDetection` - Best detection or null if none found
- **Filters**: By distance (6-60 inches) and confidence (>0.7)

#### `getAllDetections()`
Gets all valid AprilTag detections.
- **Returns**: `List<AprilTagDetection>` - All valid detections

### Telemetry Methods

#### `updateLocalizationTelemetry()`
Updates telemetry with localization information.
- **Displays**: Detection count, best detection, robot position, heading

#### `getLocalizationInfo()`
Gets detailed localization information as string.
- **Returns**: `String` - Formatted localization info

### DECODE Field Specific Methods

#### `updateRobotPositionFromAllianceGoals()`
Updates robot position using only alliance goal detections (recommended for DECODE).
- **Returns**: `boolean` - True if position was successfully updated
- **Filters**: Only uses Blue (ID 20) and Red (ID 24) alliance goals

#### `getBestAllianceGoalDetection()`
Gets the best detection from alliance goals only (excludes obelisk tags).
- **Returns**: `AprilTagDetection` - Best alliance goal detection or null

#### `isBlueAllianceGoal(AprilTagDetection detection)`
Checks if detection is from blue alliance goal.
- **Returns**: `boolean` - True if Tag ID 20

#### `isRedAllianceGoal(AprilTagDetection detection)`
Checks if detection is from red alliance goal.
- **Returns**: `boolean` - True if Tag ID 24

#### `isObeliskTag(AprilTagDetection detection)`
Checks if detection is from obelisk (not recommended for navigation).
- **Returns**: `boolean` - True if Tag IDs 21, 22, or 23

#### `getAllianceColor(AprilTagDetection detection)`
Gets the alliance color of the detected AprilTag.
- **Returns**: `String` - "Blue", "Red", "Obelisk", or "Unknown"

#### `updateDECODELocalizationTelemetry()`
Updates telemetry with DECODE-specific localization information.
- **Displays**: Alliance goal counts, obelisk tag counts, best detection details

## Configuration Parameters

### DECODE Field Detection Parameters

```java
// AprilTag specifications for DECODE field
public static final double APRILTAG_SIZE_INCHES = 8.125; // DECODE AprilTags are 8.125" square
public static final double APRILTAG_SIZE_CM = 20.65;     // ~20.65 cm square

// Detection parameters optimized for DECODE field
private final double MIN_DETECTION_DISTANCE = 6.0;    // Minimum reliable distance
private final double MAX_DETECTION_DISTANCE = 120.0;  // Maximum detection distance (increased for goal tags)
private final double MIN_DETECTION_CONFIDENCE = 0.6;  // Minimum confidence threshold (slightly lowered for goal tags)
```

### AprilTag Field Positions (Pre-configured for DECODE)

The `APRILTAG_POSITIONS` array is pre-configured for the DECODE field:
- **Blue Alliance Goal (ID 20)**: Left edge of field, center height, facing right
- **Red Alliance Goal (ID 24)**: Right edge of field, center height, facing left
- **Obelisk Tags (21, 22, 23)**: Not included (not recommended for navigation)

## Integration with Tile Navigation

The AprilTag localization system integrates seamlessly with the tile-based navigation system:

```java
// Localize using AprilTags
aprilTagNavigator.updateRobotPosition();

// Use tile-based navigation
drive.moveToTileCenter('C', 3, 0.5);
drive.moveToTabIntersection('V', 3, 0.5);

// Check current position
TileCoordinate currentPos = drive.getCurrentPosition();
String tilePos = currentPos.getTilePosition(); // "C3"
```

## Troubleshooting

### No Detections
- **Check**: Camera connection and AprilTag visibility
- **Verify**: AprilTag field positions are correctly configured
- **Adjust**: Detection distance and confidence parameters

### Poor Localization Accuracy
- **Check**: AprilTag field positions are accurate
- **Verify**: Camera calibration
- **Adjust**: Detection parameters for your setup

### Position Calculation Errors
- **Check**: AprilTag field positions array
- **Verify**: Tag IDs match your field setup
- **Debug**: Use telemetry to see detection details

## Example Autonomous OpMode

See `ExampleAprilTagLocalization.java` for a complete example showing:
- Basic localization
- Navigation based on current position
- Continuous localization during movement
- Detection quality monitoring

## Field Setup

1. **Place AprilTags**: Position AprilTags at known field locations
2. **Configure Positions**: Update `APRILTAG_POSITIONS` array with actual positions
3. **Test Detection**: Verify AprilTags are detectable from robot positions
4. **Calibrate**: Fine-tune detection parameters for your setup

## Notes

- **Coordinate System**: Uses the same field coordinate system as the tile navigation (0,0 at bottom-left)
- **Heading**: Robot heading is calculated relative to AprilTag orientation
- **Updates**: Position is automatically updated in DriveSubsystem when localized
- **Fallback**: System gracefully handles cases where no AprilTags are detected
- **Performance**: Designed for real-time localization during autonomous operation

This system provides a robust foundation for precise tile-based navigation using AprilTag localization!
