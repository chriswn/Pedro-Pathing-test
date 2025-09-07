package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import SubSystems.Drive.DriveSubsystem;
import SubSystems.Drive.TileCoordinate;
import SubSystems.Vision.AprilTagNavigator;
import SubSystems.Scoring.ShooterSubsystem;

@Autonomous(name = "FullSystemTestOpMode", group = "Test")
public class FullSystemTestOpMode extends LinearOpMode {

	@Override
	public void runOpMode() throws InterruptedException {
		// Initialize subsystems
		DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
		AprilTagNavigator april = new AprilTagNavigator(drive, hardwareMap, telemetry);
		ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);

		// Choose alliance (toggle here as needed)
		boolean isBlueAlliance = true; // set false for red

		telemetry.addLine("FullSystemTest initialized. Waiting for start...");
		telemetry.update();

		waitForStart();
		if (isStopRequested()) return;

		// Step 1: Localize with AprilTags (triangulation preferred)
		boolean localized = april.updateRobotPositionFromTriangulation();
		if (!localized) {
			localized = april.updateRobotPositionFromAllianceGoals();
		}

		TileCoordinate startPose = drive.getCurrentPosition();
		if (startPose != null) {
			telemetry.addData("Start Pose", "(%.1f, %.1f)", startPose.getX(), startPose.getY());
		}
		telemetry.update();

		// Step 2: Drive to a reasonable shot location (example: center of C3)
		drive.moveToTileCenter('C', 3, 0.5);

		// Step 3: Execute a shot based on current pose and alliance
		TileCoordinate current = drive.getCurrentPosition();
		if (current != null) {
			shooter.shootArtifact(current, isBlueAlliance);
		}

		// Hold for observation
		long endTime = System.currentTimeMillis() + 3000;
		while (opModeIsActive() && System.currentTimeMillis() < endTime) {
			april.updateDECODELocalizationTelemetry();
			telemetry.update();
			sleep(50);
		}

		// Stop shooter
		// Caller should also actuate any feeder/trigger as needed in hardware
	}
}


