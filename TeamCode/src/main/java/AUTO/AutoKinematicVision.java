package AUTO;

import android.util.Size;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import SubSystems.Arm.ArmMovement;
import SubSystems.Drive.DriveSubsystemAuto;
import SubSystems.Vision.TurboObjectDetectionPipeline;
import SubSystems.InverseKinematics;
import constants.ArmConstants;

@Autonomous(name = "AutoKinematicVision", group = "Auto")
public class AutoKinematicVision extends LinearOpMode {
    // Subsystems
    private DriveSubsystemAuto drive;
    private ArmMovement arm;

    // Vision
    private TurboObjectDetectionPipeline objectDetector;
    private VisionPortal visionPortal;
    private InverseKinematics ik;

    // Constants
    private static final double STRAFE_KP = 0.015;
    private static final double FORWARD_KP = 0.005;
    private static final int TARGET_CX = 160;
    private static final int MIN_LONG_SIDE = 200;  // Minimum pixel size for pickup
    private static final int CX_TOLERANCE = 100;
    private static final double MAX_STRAFE_ERROR = 100.0;
    private static final long ALIGN_TIMEOUT_MS = 3000;

    // Arm dimensions
    private static final double SHOULDER_LENGTH = 10.0;
    private static final double FOREARM_LENGTH = 10.0;
    private static final double SHOULDER_TICKS_PER_REV = ArmConstants.SHOULDER_TICKS_PER_REV;
    private static final double FOREARM_TICKS_PER_REV = ArmConstants.FOREARM_TICKS_PER_REV;
    private static final double MIN_SHOULDER_ANGLE = 0;
    private static final double MAX_SHOULDER_ANGLE = 135;
    private static final double MIN_FOREARM_ANGLE = 0;
    private static final double MAX_FOREARM_ANGLE = 160;

    // Camera and object parameters
    private static final double CAMERA_FORWARD_OFFSET = 5.0; // Camera's forward offset from arm base (inches)
    private static final double CAMERA_HEIGHT = 8.0;         // Camera's height above ground (inches)
    private static final double ARM_BASE_HEIGHT = 10.0;      // Arm base height above ground (inches)
    private static final double OBJECT_HEIGHT = 0.75;        // Object center height (1.5/2 = 0.75 inches)
    private static final double FOCAL_LENGTH = 500.0;        // Must match vision pipeline
    
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

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            stateMachineLoop();
            telemetry.update();
            sleep(15);  // Smooth loop
        }

        visionPortal.close();
    }

    private void initializeSystems() {
        // Initialize subsystems
        drive = new DriveSubsystemAuto(hardwareMap, telemetry);
        arm = new ArmMovement(hardwareMap, telemetry);
        ik = new InverseKinematics(  SHOULDER_LENGTH, FOREARM_LENGTH,
         MIN_SHOULDER_ANGLE,MAX_SHOULDER_ANGLE,
          MIN_FOREARM_ANGLE, MAX_FOREARM_ANGLE);

        // Initialize vision pipeline and camera
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
                telemetry.addData("Status", "Complete");
                break;
        }
    }

    private void handleSearchState() {
        if (objectDetector.isObjectDetected()) {
            currentState = RobotState.ALIGNING;
            telemetry.addData("Status", "Object detected!");
        } else {
            // Rotate slowly to search
            drive.drive(0, 0, 0.3);
        }
    }

    private void handleAlignState() {
        if (!objectDetector.isObjectDetected()) {
            currentState = RobotState.SEARCHING;
            alignStartTime = 0;
            return;
        }

        double cx = objectDetector.getCentroidX();
        double longSide = objectDetector.getLongSidePixels();
        double strafeError = cx - TARGET_CX;

        telemetry.addData("Aligning", "CentroidX: %.1f, LongSide: %.1f", cx, longSide);
        telemetry.addData("Strafe Error", "%.1f", strafeError);

        if (alignStartTime == 0) {
            alignStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - alignStartTime > ALIGN_TIMEOUT_MS) {
            telemetry.addData("Align Timeout", "Timeout exceeded. Returning to SEARCHING.");
            drive.stopMotors();
            currentState = RobotState.SEARCHING;
            alignStartTime = 0;
            return;
        }

        double strafePower = Range.clip(strafeError * STRAFE_KP, -0.4, 0.4);
        double maxForwardPower = 0.5;
        double minForwardPower = 0.1;
        double alignmentFactor = Math.max(0, 1 - (Math.abs(strafeError) / MAX_STRAFE_ERROR));
        double forwardPower = Range.clip(minForwardPower + (alignmentFactor * (maxForwardPower - minForwardPower)),
                minForwardPower, maxForwardPower);

        if (Math.abs(strafeError) < CX_TOLERANCE && longSide > MIN_LONG_SIDE) {
            telemetry.addData("Alignment Successful", "Ready to collect!");
            drive.stopMotors();
            currentState = RobotState.COLLECTING;
            alignStartTime = 0;
            executeCollectionSequence();
        } else {
            drive.drive(forwardPower, strafePower, 0);
        }
    }

    private void executeCollectionSequence() {
        double longSidePixels = objectDetector.getLongSidePixels();
        double cx = objectDetector.getCentroidX();
        
        // 1. Calculate straight-line distance to object
        double apparentDistance = (TurboObjectDetectionPipeline.SAMPLE_LENGTH * FOCAL_LENGTH) / longSidePixels;
        
        // 2. Calculate horizontal offset in inches
        double horizontalOffset = (cx - TARGET_CX) * (apparentDistance / FOCAL_LENGTH);
        
        // 3. Calculate true ground distance from camera to object
        double heightDiff = CAMERA_HEIGHT - OBJECT_HEIGHT;
        double groundDistance = Math.sqrt(apparentDistance * apparentDistance - heightDiff * heightDiff);
        
        // 4. Convert to arm base coordinates
        double targetX = horizontalOffset;
        double targetY = groundDistance + CAMERA_FORWARD_OFFSET;
        double targetZ = OBJECT_HEIGHT - ARM_BASE_HEIGHT;  // Vertical offset from arm base
        
        // 5. Calculate planar distance (radial distance in horizontal plane)
        double planarDistance = Math.sqrt(targetX * targetX + targetY * targetY);
        
        try {
            // Calculate angles using planar distance and vertical offset
            InverseKinematics.JointAngles angles = ik.calculateJointAngles(planarDistance, targetZ);

            // Convert angles to encoder ticks
            int shoulderTicks = (int) InverseKinematics.degreesToTicks(angles.shoulderAngle, SHOULDER_TICKS_PER_REV);
            int forearmTicks = (int) InverseKinematics.degreesToTicks(angles.forearmAngle, FOREARM_TICKS_PER_REV);

            shoulderTicks += ArmConstants.SHOULDER_BACKLASH_COMP;
            forearmTicks += ArmConstants.FOREARM_BACKLASH_COMP;

            telemetry.addData("Target Position", "X: %.1f, Y: %.1f, Z: %.1f", targetX, targetY, targetZ);
            telemetry.addData("Shoulder Ticks", shoulderTicks);
            telemetry.addData("Forearm Ticks", forearmTicks);

            // Move arm
            arm.moveArmToPosition(shoulderTicks, forearmTicks);
            sleep(500); // Wait for arm movement

            // Gripper operations
            arm.openGripper();
            sleep(100);
            arm.closeGripper();
            sleep(100);

            // Retract arm
            arm.moveShoulderToPosition(ArmConstants.SHOULDER_HOME_POSITION);
            arm.rotateForearmToAngle(ArmConstants.FOREARM_HOME_POSITION);
            sleep(500);
            arm.openGripper();

            currentState = RobotState.RETRACTING;
        } catch (Exception e) {
            telemetry.addData("IK Error", e.getMessage());
            currentState = RobotState.SEARCHING;
        }
    }

    private void handleCollectState() {
        // Collection done in executeCollectionSequence, no action here
    }

    private void handleRetractState() {
        drive.drive(-0.3, 0, 0);
        sleep(700);
        drive.stopMotors();
        currentState = RobotState.COMPLETE;
    }
}