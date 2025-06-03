package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import SubSystems.Drive.DriveSubsystem;
import SubSystems.Drive.TileCoordinate;
import SubSystems.Vision.AprilTagNavigator;

/**
 * Example autonomous opmode demonstrating AprilTag-based localization for
 * DECODE field.
 * This example shows how to use AprilTag detection from alliance goals to
 * determine
 * the robot's current tile position and navigate accordingly.
 * 
 * Based on FTC Competition Manual Section 9.10 AprilTags:
 * - Blue Alliance Goal: Tag ID 20
 * - Red Alliance Goal: Tag ID 24
 * - Obelisk Tags (21, 22, 23): NOT recommended for navigation
 * 
 * @author Pedro Pathing Team
 * @version 1.0
 */
@Autonomous(name = "Example DECODE Localization", group = "Examples")
public class ExampleDECODELocalization extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator aprilTagNavigator;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTagNavigator = new AprilTagNavigator(drive, hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized - DECODE AprilTag Localization");
        telemetry.addData("AprilTag Size", "%.1f inches (%.1f cm)",
                AprilTagNavigator.APRILTAG_SIZE_INCHES, AprilTagNavigator.APRILTAG_SIZE_CM);
        telemetry.addData("Target Tags", "Blue Goal (ID 20), Red Goal (ID 24)");
        telemetry.addData("Instructions", "Place robot on field with alliance goals visible");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Example 1: Basic DECODE localization using alliance goals
            telemetry.addData("Action", "Attempting DECODE localization...");
            telemetry.update();

            // Try to localize using alliance goals for up to 5 seconds
            ElapsedTime localizationTimer = new ElapsedTime();
            boolean localized = false;

            while (opModeIsActive() && !localized && localizationTimer.seconds() < 5.0) {
                localized = aprilTagNavigator.updateRobotPositionFromAllianceGoals();
                aprilTagNavigator.updateDECODELocalizationTelemetry();
                telemetry.update();
                sleep(100);
            }

            if (localized) {
                telemetry.addData("Localization", "SUCCESS!");
                demonstrateDECODENavigation();
            } else {
                telemetry.addData("Localization", "FAILED - No alliance goals detected");
                telemetry.addData("Action", "Using default position and continuing...");
                telemetry.update();
                sleep(2000);

                // Fallback: Set a default position based on alliance
                setDefaultPositionBasedOnAlliance();
            }

            // Example 2: Continuous DECODE localization
            telemetry.addData("Action", "Demonstrating continuous DECODE localization...");
            telemetry.update();
            demonstrateContinuousDECODELocalization();

            // Example 3: Show DECODE-specific information
            telemetry.addData("Action", "Showing DECODE localization info...");
            telemetry.update();
            showDECODELocalizationInfo();

            telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
            sleep(3000);
        }

        aprilTagNavigator.closeVision();
    }

    /**
     * Demonstrate navigation based on detected alliance goal
     */
    private void demonstrateDECODENavigation() {
        TileCoordinate currentPos = drive.getCurrentPosition();
        char currentTile = currentPos.getTileColumn();
        int currentRow = currentPos.getTileRow();

        telemetry.addData("Navigation", "Current position: %s", currentPos.getTilePosition());
        telemetry.update();

        // Determine which alliance goal was detected and navigate accordingly
        var bestDetection = aprilTagNavigator.getBestAllianceGoalDetection();
        if (bestDetection != null) {
            String alliance = aprilTagNavigator.getAllianceColor(bestDetection);
            telemetry.addData("Detected Alliance", alliance);

            if (aprilTagNavigator.isBlueAllianceGoal(bestDetection)) {
                // Blue alliance detected - navigate from blue side
                telemetry.addData("Action", "Blue alliance detected - navigating from blue side");
                telemetry.update();
                navigateFromBlueAlliance();
            } else if (aprilTagNavigator.isRedAllianceGoal(bestDetection)) {
                // Red alliance detected - navigate from red side
                telemetry.addData("Action", "Red alliance detected - navigating from red side");
                telemetry.update();
                navigateFromRedAlliance();
            }
        }
    }

    /**
     * Navigate from blue alliance side
     */
    private void navigateFromBlueAlliance() {
        // Move towards center of field
        drive.moveToTileCenter('C', 3, 0.3);
        sleep(2000);

        // Move to scoring position
        drive.moveToTile('D', 4, 12, 12, 0.3);
        sleep(2000);

        // Return to blue side
        drive.moveToTile('B', 2, 12, 12, 0.3);
        sleep(2000);
    }

    /**
     * Navigate from red alliance side
     */
    private void navigateFromRedAlliance() {
        // Move towards center of field
        drive.moveToTileCenter('C', 3, 0.3);
        sleep(2000);

        // Move to scoring position
        drive.moveToTile('D', 4, 12, 12, 0.3);
        sleep(2000);

        // Return to red side
        drive.moveToTile('E', 2, 12, 12, 0.3);
        sleep(2000);
    }

    /**
     * Set default position based on detected alliance
     */
    private void setDefaultPositionBasedOnAlliance() {
        var detections = aprilTagNavigator.getAllDetections();

        // Check if we can determine alliance from any detection
        boolean blueDetected = detections.stream().anyMatch(aprilTagNavigator::isBlueAllianceGoal);
        boolean redDetected = detections.stream().anyMatch(aprilTagNavigator::isRedAllianceGoal);

        if (blueDetected) {
            // Set position on blue side
            drive.setPosition(new TileCoordinate('A', 3, 12, 12));
            drive.setHeading(Math.toRadians(0)); // Facing right
            telemetry.addData("Default Position", "Blue side - A3");
        } else if (redDetected) {
            // Set position on red side
            drive.setPosition(new TileCoordinate('F', 3, 12, 12));
            drive.setHeading(Math.toRadians(180)); // Facing left
            telemetry.addData("Default Position", "Red side - F3");
        } else {
            // Default to center
            drive.setPosition(new TileCoordinate('C', 3, 12, 12));
            drive.setHeading(Math.toRadians(0));
            telemetry.addData("Default Position", "Center - C3");
        }
        telemetry.update();
    }

    /**
     * Demonstrate continuous localization during movement
     */
    private void demonstrateContinuousDECODELocalization() {
        // Move to different positions while continuously localizing
        TileCoordinate[] waypoints = {
                new TileCoordinate('B', 2, 12, 12),
                new TileCoordinate('C', 3, 12, 12),
                new TileCoordinate('D', 4, 12, 12),
                new TileCoordinate('E', 5, 12, 12)
        };

        for (int i = 0; i < waypoints.length && opModeIsActive(); i++) {
            TileCoordinate target = waypoints[i];

            telemetry.addData("Action", "Moving to %s", target.getTilePosition());
            telemetry.update();

            // Move to target
            drive.moveToPosition(target, 0.3);
            sleep(1000);

            // Update localization using alliance goals
            boolean localized = aprilTagNavigator.updateRobotPositionFromAllianceGoals();
            if (localized) {
                TileCoordinate actualPos = drive.getCurrentPosition();
                telemetry.addData("Localized Position", actualPos.getTilePosition());
                telemetry.addData("Target Position", target.getTilePosition());
                telemetry.addData("Distance Error", "%.1f inches", actualPos.distanceTo(target));

                // Show which alliance goal was used
                var bestDetection = aprilTagNavigator.getBestAllianceGoalDetection();
                if (bestDetection != null) {
                    String alliance = aprilTagNavigator.getAllianceColor(bestDetection);
                    telemetry.addData("Using Alliance Goal", alliance);
                }
            } else {
                telemetry.addData("Localization", "Lost during movement");
            }
            telemetry.update();
            sleep(1000);
        }
    }

    /**
     * Show detailed DECODE-specific localization information
     */
    private void showDECODELocalizationInfo() {
        // Show all detected AprilTags with their types
        var detections = aprilTagNavigator.getAllDetections();
        telemetry.addData("Total Detections", detections.size());

        long blueGoals = detections.stream().filter(aprilTagNavigator::isBlueAllianceGoal).count();
        long redGoals = detections.stream().filter(aprilTagNavigator::isRedAllianceGoal).count();
        long obeliskTags = detections.stream().filter(aprilTagNavigator::isObeliskTag).count();

        telemetry.addData("Blue Alliance Goals", blueGoals);
        telemetry.addData("Red Alliance Goals", redGoals);
        telemetry.addData("Obelisk Tags", obeliskTags);
        telemetry.addData("Other Tags", detections.size() - blueGoals - redGoals - obeliskTags);

        // Show detection details
        for (var detection : detections) {
            String type = aprilTagNavigator.getAllianceColor(detection);
            String recommended = (type.equals("Blue") || type.equals("Red")) ? "✓" : "✗";
            telemetry.addData("Tag %d (%s)", "Range: %.1f, Confidence: %.2f %s",
                    detection.id, type, detection.ftcPose.range,
                    detection.decisionMargin, recommended);
        }

        // Show current position details
        TileCoordinate currentPos = drive.getCurrentPosition();
        if (currentPos != null) {
            double[] offset = currentPos.getTileOffset();
            telemetry.addData("Current Position", "Tile: %s, Tab: %s",
                    currentPos.getTilePosition(), currentPos.getTabPosition());
            telemetry.addData("Tile Offset", "X: %.1f, Y: %.1f", offset[0], offset[1]);
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
        }
    }

    /**
     * Demonstrate alliance-specific strategies
     */
    private void demonstrateAllianceStrategies() {
        var bestDetection = aprilTagNavigator.getBestAllianceGoalDetection();
        if (bestDetection == null)
            return;

        if (aprilTagNavigator.isBlueAllianceGoal(bestDetection)) {
            telemetry.addData("Strategy", "Blue Alliance - Focus on left side of field");
            // Blue alliance specific navigation
        } else if (aprilTagNavigator.isRedAllianceGoal(bestDetection)) {
            telemetry.addData("Strategy", "Red Alliance - Focus on right side of field");
            // Red alliance specific navigation
        }
    }
}
