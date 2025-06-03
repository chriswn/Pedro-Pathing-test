package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import SubSystems.Drive.DriveSubsystem;
import SubSystems.Drive.TileCoordinate;

/**
 * Example autonomous opmode demonstrating tile-based navigation.
 * This example shows how to use the tile coordinate system for field
 * navigation.
 * 
 * @author Pedro Pathing Team
 * @version 1.0
 */
@Autonomous(name = "Example Tile Navigation", group = "Examples")
public class ExampleTileNavigation extends LinearOpMode {

    private DriveSubsystem drive;
    private ElapsedTime runtime;

    // Example field positions using tile coordinates
    private TileCoordinate startPosition;
    private TileCoordinate scoringPosition;
    private TileCoordinate pickupPosition;
    private TileCoordinate parkPosition;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        drive = new DriveSubsystem(hardwareMap, telemetry);
        runtime = new ElapsedTime();

        // Define field positions using tile coordinates
        // These correspond to typical FTC field positions
        startPosition = new TileCoordinate('A', 1, 12, 12); // Start in A1 with offset
        scoringPosition = new TileCoordinate('C', 3, 12, 12); // Score in C3
        pickupPosition = new TileCoordinate('E', 2, 12, 12); // Pickup in E2
        parkPosition = new TileCoordinate('F', 6, 12, 12); // Park in F6

        // Set initial position
        drive.setPosition(startPosition);
        drive.setHeading(Math.toRadians(0)); // Facing right

        telemetry.addData("Status", "Initialized - Tile Navigation Example");
        telemetry.addData("Start Position", startPosition.getTilePosition());
        telemetry.addData("Field Size", "%.0f x %.0f inches", TileCoordinate.FIELD_WIDTH, TileCoordinate.FIELD_HEIGHT);
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Example 1: Move to center of a tile
            telemetry.addData("Action", "Moving to tile center C3");
            telemetry.update();
            drive.moveToTileCenter('C', 3, 0.5);
            sleep(2000);

            // Example 2: Move to specific position within a tile
            telemetry.addData("Action", "Moving to specific position in E2");
            telemetry.update();
            drive.moveToTile('E', 2, 18, 6, 0.5); // 18 inches right, 6 inches up from tile corner
            sleep(2000);

            // Example 3: Move to tab-line intersection
            telemetry.addData("Action", "Moving to tab intersection V3");
            telemetry.update();
            drive.moveToTabIntersection('V', 3, 0.5);
            sleep(2000);

            // Example 4: Relative movement by tiles
            telemetry.addData("Action", "Moving 2 tiles forward, 1 tile right");
            telemetry.update();
            drive.moveRelativeTiles(2, 1, 0.5);
            sleep(2000);

            // Example 5: Turn to face a specific tile
            telemetry.addData("Action", "Turning to face tile F6");
            telemetry.update();
            drive.turnToTile(parkPosition, 0.3);
            sleep(2000);

            // Example 6: Move to final park position
            telemetry.addData("Action", "Moving to park position F6");
            telemetry.update();
            drive.moveToPosition(parkPosition, 0.5);
            sleep(2000);

            // Display final position information
            telemetry.addData("Final Position", drive.getCurrentPosition().getTilePosition());
            telemetry.addData("Final Tab", drive.getCurrentPosition().getTabPosition());
            telemetry.addData("Total Time", "%.1f seconds", runtime.seconds());
            telemetry.update();

            // Keep displaying telemetry for a few seconds
            sleep(3000);
        }

        drive.stop();
    }

    /**
     * Helper method to demonstrate different tile coordinate creation methods
     */
    private void demonstrateTileCoordinateCreation() {
        // Method 1: From field position in inches
        TileCoordinate pos1 = new TileCoordinate(48, 72); // 48 inches right, 72 inches up

        // Method 2: From tile grid with offset
        TileCoordinate pos2 = new TileCoordinate('C', 3, 12, 6); // C3 tile, 12" right, 6" up from corner

        // Method 3: From tab-line intersection
        TileCoordinate pos3 = new TileCoordinate('X', 3); // X3 tab intersection

        telemetry.addData("Position 1", pos1.toString());
        telemetry.addData("Position 2", pos2.toString());
        telemetry.addData("Position 3", pos3.toString());
    }

    /**
     * Helper method to demonstrate tile coordinate utilities
     */
    private void demonstrateTileUtilities() {
        TileCoordinate current = drive.getCurrentPosition();
        TileCoordinate target = new TileCoordinate('F', 6);

        double distance = current.distanceTo(target);
        double angle = current.angleTo(target);

        telemetry.addData("Distance to F6", "%.1f inches", distance);
        telemetry.addData("Angle to F6", "%.1f degrees", Math.toDegrees(angle));
        telemetry.addData("Is at F6?", drive.isAtTile(target, 6.0)); // Within 6 inches
    }
}
