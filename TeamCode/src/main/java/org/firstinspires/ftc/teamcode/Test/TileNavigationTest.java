package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import SubSystems.Drive.DriveSubsystem;
import SubSystems.Drive.TileCoordinate;
import SubSystems.Vision.AprilTagNavigator;

/**
 * Test OpMode demonstrating the integration of Tile-based navigation and AprilTag localization
 */
@Autonomous(name = "Test: Tile Navigation + AprilTag", group = "Test")
public class TileNavigationTest extends LinearOpMode {

    private DriveSubsystem drive;
    private AprilTagNavigator aprilTagNav;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTagNav = new AprilTagNavigator(drive, hardwareMap, telemetry);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized - Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();

        try {
            // Test 1: Initial Localization
            telemetry.addData("Test", "1. Initial Localization");
            telemetry.update();

            boolean localized = false;
            int attempts = 0;
            while (!localized && attempts < 3 && opModeIsActive()) {
                localized = aprilTagNav.updateRobotPositionFromAllianceGoals();
                if (!localized) {
                    telemetry.addData("Status", "Attempting to localize... Attempt " + (attempts + 1));
                    telemetry.update();
                    sleep(1000);
                }
                attempts++;
            }

            if (!localized) {
                telemetry.addData("Error", "Failed to localize using AprilTags");
                telemetry.update();
                return;
            }

            // Test 2: Navigation to Different Tiles
            telemetry.addData("Test", "2. Tile Navigation");
            telemetry.update();

            // Move to center of tile B2
            drive.moveToTile('B', 2, 12, 12, 0.5);
            sleep(1000);

            // Move to tab intersection W3
            drive.moveToTabIntersection('W', 3, 0.5);
            sleep(1000);

            // Test 3: Alliance-specific behavior
            telemetry.addData("Test", "3. Alliance Detection");
            telemetry.update();

            AprilTagDetection bestDetection = aprilTagNav.getBestAllianceGoalDetection();
            if (bestDetection != null) {
                if (aprilTagNav.isBlueAllianceGoal(bestDetection)) {
                    // Blue alliance specific path
                    telemetry.addData("Alliance", "Blue - Moving to C3");
                    drive.moveToTile('C', 3, 12, 12, 0.5);
                } else if (aprilTagNav.isRedAllianceGoal(bestDetection)) {
                    // Red alliance specific path
                    telemetry.addData("Alliance", "Red - Moving to D3");
                    drive.moveToTile('D', 3, 12, 12, 0.5);
                }
            }

            // Test 4: Position Reporting
            telemetry.addData("Test", "4. Position Reporting");
            TileCoordinate currentPos = drive.getCurrentPosition();
            telemetry.addData("Final Position", "Tile: %s, Tab: %s",
                    currentPos.getTilePosition(),
                    currentPos.getTabPosition());
            telemetry.addData("Field Position", "X: %.1f, Y: %.1f",
                    currentPos.getX(),
                    currentPos.getY());

            // Test 5: Distance Calculations
            telemetry.addData("Test", "5. Distance Calculations");
            TileCoordinate targetPos = new TileCoordinate('E', 4, 12, 12);
            double distanceToTarget = currentPos.distanceTo(targetPos);
            double angleToTarget = Math.toDegrees(currentPos.angleTo(targetPos));
            telemetry.addData("To Target E4", "Distance: %.1f in, Angle: %.1f deg",
                    distanceToTarget, angleToTarget);

        } catch (Exception e) {
            telemetry.addData("Error", "Exception: " + e.getMessage());
        } finally {
            // Clean up
            aprilTagNav.closeVision();

            telemetry.addData("Status", "Test Complete");
            telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
            telemetry.update();

            while (opModeIsActive()) {
                // Keep the telemetry visible
                sleep(100);
            }
        }
    }
}
