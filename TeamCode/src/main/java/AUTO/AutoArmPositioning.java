package AUTO;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import SubSystems.Drive.DriveSubsystemAuto;
import SubSystems.Vision.TurboObjectDetectionPipeline;
import SubSystems.Arm.ArmMovement;

@Autonomous(name = "Auto Pickup with Arm Positioning", group = "Competition")
public class AutoArmPositioning extends LinearOpMode {
    // Subsystems
    private DriveSubsystemAuto drive;
    private ArmMovement arm;

    // Vision
    private TurboObjectDetectionPipeline objectDetector;
    private VisionPortal visionPortal;

    // Constants
    private static final double STRAFE_KP = 0.008;
    private static final double FORWARD_KP = 0.003;
    private static final int TARGET_CX = 340;
    private static final int MIN_WIDTH_FOR_PICKUP = 200;
    private static final int CX_TOLERANCE = 100;
    private static final double ARM_POSITION_OFFSET = 100; // Offset for arm position adjustment

    // State tracking
    private enum RobotState {
        SEARCHING, ALIGNING, COLLECTING, RETRACTING, COMPLETE
    }
    private RobotState currentState = RobotState.SEARCHING;

    @Override
    public void runOpMode() {
        initializeSystems();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                stateMachineLoop();
                updateTelemetry();
                sleep(20); // Keep loop running smoothly
            }
        }

        visionPortal.close();
    }

    private void initializeSystems() {
        // Initialize subsystems
        drive = new DriveSubsystemAuto(hardwareMap, telemetry);
        arm = new ArmMovement(hardwareMap, telemetry);

        // Initialize vision pipeline
        objectDetector = new TurboObjectDetectionPipeline();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(objectDetector)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        // Arm initialization sequence
        arm.moveShoulderToPosition(100); // Engage encoders
        sleep(500);
        arm.stopMotors();
        arm.closeGripper();
        arm.openGripper();
        arm.Gripper();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void stateMachineLoop() {
        switch (currentState) {
            case SEARCHING:
                handleSearchState();
                break;
            case ALIGNING:
                handleAlignState();
                break;
            case COLLECTING:
                handleCollectState();
                break;
            case RETRACTING:
                handleRetractState();
                break;
            case COMPLETE:
                drive.stopMotors();
                break;
        }
    }

    private void handleSearchState() {
        if (objectDetector.isObjectDetected()) {
            currentState = RobotState.ALIGNING;
            telemetry.addData("Status", "Object detected!");
        } else {
            // Perform search pattern (slow rotate)
            drive.drive(0, 0, 0.3); // Rotate slowly
        }
    }

    private void handleAlignState() {
        if (!objectDetector.isObjectDetected()) {
            currentState = RobotState.SEARCHING;  // Object not found, go back to search
            return;
        }

        double cx = objectDetector.getCentroidX();  // X position of the object
        double width = objectDetector.getWidth();  // Width of the detected object

        // Calculate alignment errors
        double strafeError = cx - TARGET_CX;  // Error in X axis alignment

        telemetry.addData("Aligning", "CentroidX: %.1f, Width: %.1f", cx, width);
        telemetry.addData("strafeError", "%.1f", strafeError);

        // Calculate motor power for strafe (side-to-side) alignment
        double strafePower = strafeError * STRAFE_KP;

        // Clamp values to avoid excessive movement or rotation
        strafePower = Range.clip(strafePower, -0.3, 0.3);

        // Define constant forward power for moving forward
        double forwardPower = 0.4;  // Constant forward speed

        // If aligned within tolerance, and the object is large enough, stop and start collecting
        if (Math.abs(strafeError) < CX_TOLERANCE && width > MIN_WIDTH_FOR_PICKUP) {
            telemetry.addData("Alignment Successful", "Ready to collect!");
            drive.stopMotors();  // Stop movement
            currentState = RobotState.COLLECTING;
            executeCollectionSequence(width);  // Pass width to adjust arm
        } else {
            // Drive forward with constant speed and adjust strafe for alignment
            drive.drive(forwardPower, strafePower, 0);  // Only adjusting strafePower for alignment
        }

        telemetry.update();  // Update telemetry
    }


    private void executeCollectionSequence(double objectWidth) {
        // Get distance to the object
        double objectDistance = objectDetector.calculateDistance(objectWidth);  // Use the calculateDistance function from the vision pipeline

        // Adjust arm based on the detected object size and distance
        adjustArmPositionBasedOnObject(objectWidth);

        // Move forearm first to ensure clearance for shoulder movement
        arm.rotateForearmToAngle(360);
        sleep(800); // Wait for the forearm to move into position

        // Lower shoulder to the collection position (if possible)
        arm.moveShoulderToPosition(-350);
        sleep(1000); // Allow time for the shoulder to move down

        // Final approach to the object
        drive.drive(0.15, 0, 0);
        sleep(300);
        arm.openGripper();
        drive.stopMotors();

        // Open and close gripper to collect the object
        arm.openGripper();
        sleep(500);
        arm.closeGripper();
        sleep(800);

        arm.moveShoulderToPosition(500);
        arm.rotateForearmToAngle(-500); // Adjust forearm for better stability
        sleep(1000);

        arm.openGripper();

        // Transition to retracting state
        currentState = RobotState.RETRACTING;
    }

    private void handleCollectState() {
        // Collection process already handled in executeCollectionSequence
    }

    private void handleRetractState() {
        // Back away from collection area
        drive.drive(-0.3, 0, 0);
        sleep(1000);
        drive.stopMotors();
        currentState = RobotState.COMPLETE;
    }

    private void adjustArmPositionBasedOnObject(double objectWidth) {
        // Calculate the target arm position based on object width
        double targetArmPosition = calculateArmPosition(objectWidth);

        // Move arm to target position
        arm.moveShoulderToPosition((int) targetArmPosition); // Arm movement in encoder positions
    }

    private double calculateArmPosition(double objectWidth) {
        // Simple scaling factor to adjust arm position based on object width
        double position = 0;

        if (objectWidth < MIN_WIDTH_FOR_PICKUP) {
            position = ARM_POSITION_OFFSET; // Lower arm if object is small/close
        } else if (objectWidth >= MIN_WIDTH_FOR_PICKUP && objectWidth < 500) {
            position = ARM_POSITION_OFFSET + 100; // Slightly higher for medium objects
        } else {
            position = ARM_POSITION_OFFSET + 200; // Even higher for larger objects
        }

        // Clamp values to prevent moving beyond physical limits
        position = Range.clip(position, 0, 1000);

        telemetry.addData("Target Arm Position", "%.1f", position);
        return position;
    }

    private void updateTelemetry() {
        telemetry.update();
    }
}
