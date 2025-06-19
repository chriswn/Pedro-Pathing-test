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
    private static final double STRAFE_KP = 0.015;
    private static final double FORWARD_KP = 0.005;
    private static final int TARGET_CX = 340;
    private static final int MIN_WIDTH_FOR_PICKUP = 200;
    private static final int CX_TOLERANCE = 100;
    private static final double ARM_POSITION_OFFSET = 100; // Offset for arm position adjustment
    private static final double MAX_STRAFE_ERROR = 100.0;  // Tune based on your camera/resolution
    private static final long ALIGN_TIMEOUT_MS = 3000;     
    private long alignStartTime = 0;  

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
                sleep(15); // Keep loop running smoothly
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
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        // Arm initialization sequence
        arm.moveShoulderToPosition(100); // Engage encoders
        sleep(100);
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
        currentState = RobotState.SEARCHING;
        alignStartTime = 0;  // Reset timer
        return;
    }

    double cx = objectDetector.getCentroidX();
    double width = objectDetector.getWidth();

    double strafeError = cx - TARGET_CX;

    telemetry.addData("Aligning", "CentroidX: %.1f, Width: %.1f", cx, width);
    telemetry.addData("strafeError", "%.1f", strafeError);
    telemetry.addData("Debug", "TARGET_CX: %.1f, STRAFE_KP: %.3f", TARGET_CX, STRAFE_KP);

    if (STRAFE_KP <= 0 || STRAFE_KP > 0.1) {
        telemetry.addData("Warning", "Check STRAFE_KP value: %.3f", STRAFE_KP);
    }

    // Start the timer if it's the first call to this state
    if (alignStartTime == 0) {
        alignStartTime = System.currentTimeMillis();
    }

    // Check for alignment timeout
    if (System.currentTimeMillis() - alignStartTime > ALIGN_TIMEOUT_MS) {
        telemetry.addData("Align Timeout", "Timeout exceeded. Reverting to SEARCHING.");
        drive.stopMotors();
        currentState = RobotState.SEARCHING;
        alignStartTime = 0;
        telemetry.update();
        return;
    }

    // Calculate motor power for strafing
    double strafePower = Range.clip(strafeError * STRAFE_KP, -0.4, 0.4);

    // Dynamic forward power based on alignment
    double maxForwardPower = 0.5;
    double minForwardPower = 0.1;
    double alignmentFactor = Math.max(0, 1 - (Math.abs(strafeError) / MAX_STRAFE_ERROR));
    double forwardPower = Range.clip(minForwardPower + (alignmentFactor * (maxForwardPower - minForwardPower)),
                                     minForwardPower, maxForwardPower);

    // Check if aligned and object is large enough
    if (Math.abs(strafeError) < CX_TOLERANCE && width > MIN_WIDTH_FOR_PICKUP) {
        telemetry.addData("Alignment Successful", "Ready to collect!");
        drive.stopMotors();
        currentState = RobotState.COLLECTING;
        alignStartTime = 0;  // Reset timer
        executeCollectionSequence(width);
    } else {
        // Continue driving forward with strafe correction
        drive.drive(forwardPower, strafePower, 0);
    }

    telemetry.update();
}


    private void executeCollectionSequence(double objectWidth) {
        // Get distance to the object
        double objectDistance = objectDetector.calculateDistance(objectWidth, objectDetector.getAspectRatio());  // Use the calculateDistance function from the vision pipeline

        // Adjust arm based on the detected object size and distance
        adjustArmPositionBasedOnObject(objectWidth);

        // Move forearm first to ensure clearance for shoulder movement
        arm.rotateForearmToAngle(360);
        sleep(250); // Wait for the forearm to move into position

        // Lower shoulder to the collection position (if possible)
        arm.moveShoulderToPosition(-350);
        sleep(290); // Allow time for the shoulder to move down

        // Final approach to the object
        drive.drive(0.15, 0, 0);
        sleep(150);
        arm.openGripper();
        drive.stopMotors();

        // Open and close gripper to collect the object
        sleep(50);
        arm.closeGripper();
        sleep(100);

        arm.moveShoulderToPosition(500);
        arm.rotateForearmToAngle(-500); 

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
        sleep(700);
        drive.stopMotors();
        currentState = RobotState.COMPLETE;
    }

    private void adjustArmPositionBasedOnObject(double objectWidth) {
        // Calculate the target arm position based on object width
        double targetArmPosition = calculateArmPosition(objectWidth);

        // Move arm to target position
        arm.moveShoulderToPosition((int) targetArmPosition); 
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
        position = Range.clip(position, 0, 800);

        telemetry.addData("Target Arm Position", "%.1f", position);
        return position;
    }

    private void updateTelemetry() {
        telemetry.update();
    }
}
