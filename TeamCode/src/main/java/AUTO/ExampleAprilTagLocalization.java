package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import SubSystems.Drive.DriveSubsystem;
import SubSystems.Drive.TileCoordinate;
import SubSystems.Vision.AprilTagNavigator;

/**
 * Example autonomous opmode demonstrating AprilTag-based tile localization.
 * This example shows how to use AprilTag detection to determine the robot's
 * current tile position and navigate accordingly.
 * 
 * @author Pedro Pathing Team
 * @version 1.0
 */
@Autonomous(name = "Example AprilTag Localization", group = "Examples")
public class ExampleAprilTagLocalization extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator aprilTagNavigator;
    private ElapsedTime runtime;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTagNavigator = new AprilTagNavigator(drive, hardwareMap, telemetry);
        runtime = new ElapsedTime();

        telemetry.addData("Status", "Initialized - AprilTag Localization Example");
        telemetry.addData("Instructions", "Place robot on field with AprilTags visible");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Example 1: Basic localization
            telemetry.addData("Action", "Attempting AprilTag localization...");
            telemetry.update();

            // Try to localize for up to 5 seconds
            ElapsedTime localizationTimer = new ElapsedTime();
            boolean localized = false;

            while (opModeIsActive() && !localized && localizationTimer.seconds() < 5.0) {
                localized = aprilTagNavigator.updateRobotPosition();
                aprilTagNavigator.updateLocalizationTelemetry();
                telemetry.update();
                sleep(100);
            }

            if (localized) {
                telemetry.addData("Localization", "SUCCESS!");
                TileCoordinate currentPos = drive.getCurrentPosition();
                telemetry.addData("Current Tile", currentPos.getTilePosition());
                telemetry.addData("Current Tab", currentPos.getTabPosition());
                telemetry.addData("Field Position", "X: %.1f, Y: %.1f", currentPos.getX(), currentPos.getY());
                telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(drive.getCurrentHeading()));
                telemetry.update();
                sleep(2000);

                // Example 2: Navigate based on current position
                demonstrateNavigationBasedOnPosition();

            } else {
                telemetry.addData("Localization", "FAILED - No AprilTags detected");
                telemetry.addData("Action", "Using default position and continuing...");
                telemetry.update();
                sleep(2000);

                // Fallback: Set a default position
                drive.setPosition(new TileCoordinate('A', 1, 12, 12));
                drive.setHeading(Math.toRadians(0));
            }

            // Example 3: Continuous localization during movement
            telemetry.addData("Action", "Demonstrating continuous localization...");
            telemetry.update();
            demonstrateContinuousLocalization();

            // Example 4: Show localization information
            telemetry.addData("Action", "Showing localization info...");
            telemetry.update();
            showLocalizationInfo();

            telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
            telemetry.update();
            sleep(3000);
        }

        aprilTagNavigator.closeVision();
    }

    /**
     * Demonstrate navigation based on current localized position
     */
    private void demonstrateNavigationBasedOnPosition() {
        TileCoordinate currentPos = drive.getCurrentPosition();
        char currentTile = currentPos.getTileColumn();
        int currentRow = currentPos.getTileRow();

        telemetry.addData("Navigation", "Current position: %c%d", currentTile, currentRow);
        telemetry.update();

        // Navigate based on current tile
        if (currentTile == 'A') {
            // If in column A, move to column C
            telemetry.addData("Action", "Moving from A to C");
            telemetry.update();
            drive.moveToTileCenter('C', currentRow, 0.3);
            sleep(2000);
        } else if (currentTile == 'C') {
            // If in column C, move to column E
            telemetry.addData("Action", "Moving from C to E");
            telemetry.update();
            drive.moveToTileCenter('E', currentRow, 0.3);
            sleep(2000);
        } else {
            // If in other columns, move to center of field
            telemetry.addData("Action", "Moving to field center");
            telemetry.update();
            drive.moveToTileCenter('C', 3, 0.3);
            sleep(2000);
        }
    }

    /**
     * Demonstrate continuous localization during movement
     */
    private void demonstrateContinuousLocalization() {
        // Move to a few different positions while continuously localizing
        TileCoordinate[] waypoints = {
                new TileCoordinate('B', 2, 12, 12),
                new TileCoordinate('D', 4, 12, 12),
                new TileCoordinate('F', 6, 12, 12)
        };

        for (int i = 0; i < waypoints.length && opModeIsActive(); i++) {
            TileCoordinate target = waypoints[i];

            telemetry.addData("Action", "Moving to %s", target.getTilePosition());
            telemetry.update();

            // Move to target
            drive.moveToPosition(target, 0.3);
            sleep(1000);

            // Update localization
            boolean localized = aprilTagNavigator.updateRobotPosition();
            if (localized) {
                TileCoordinate actualPos = drive.getCurrentPosition();
                telemetry.addData("Localized Position", actualPos.getTilePosition());
                telemetry.addData("Target Position", target.getTilePosition());
                telemetry.addData("Distance Error", "%.1f inches", actualPos.distanceTo(target));
            } else {
                telemetry.addData("Localization", "Lost during movement");
            }
            telemetry.update();
            sleep(1000);
        }
    }

    /**
     * Show detailed localization information
     */
    private void showLocalizationInfo() {
        // Show all detected AprilTags
        java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections = aprilTagNavigator.getAllDetections();
        telemetry.addData("Detected Tags", detections.size());

        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : detections) {
            telemetry.addData("Tag %d", "Range: %.1f, Confidence: %.2f",
                    detection.id, detection.ftcPose.range, detection.decisionMargin);
        }

        // Show localization status
        telemetry.addData("Localization Status", aprilTagNavigator.isLocalized() ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Localization Info", aprilTagNavigator.getLocalizationInfo());

        // Show current position details
        TileCoordinate currentPos = drive.getCurrentPosition();
        if (currentPos != null) {
            double[] offset = currentPos.getTileOffset();
            telemetry.addData("Tile Details", "Tile: %s, Offset: [%.1f, %.1f]",
                    currentPos.getTilePosition(), offset[0], offset[1]);
        }
    }

    /**
     * Helper method to demonstrate different AprilTag detection scenarios
     */
    private void demonstrateDetectionScenarios() {
        telemetry.addData("Detection Scenarios", "Testing different detection conditions");
        telemetry.update();

        // Test with different detection parameters
        org.firstinspires.ftc.vision.apriltag.AprilTagDetection bestDetection = aprilTagNavigator.getBestDetection();
        if (bestDetection != null) {
            telemetry.addData("Best Detection", "ID: %d, Range: %.1f",
                    bestDetection.id, bestDetection.ftcPose.range);
        }

        java.util.List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> allDetections = aprilTagNavigator.getAllDetections();
        telemetry.addData("All Valid Detections", allDetections.size());

        // Show detection quality
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : allDetections) {
            double quality = detection.decisionMargin;
            String qualityStr = quality > 0.9 ? "Excellent" : quality > 0.7 ? "Good" : quality > 0.5 ? "Fair" : "Poor";
            telemetry.addData("Tag %d Quality", "%s (%.2f)", detection.id, qualityStr, quality);
        }
    }
}
