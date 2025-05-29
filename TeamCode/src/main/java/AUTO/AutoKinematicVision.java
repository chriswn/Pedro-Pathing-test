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
    private static final int MIN_WIDTH_FOR_PICKUP = 200;
    private static final int CX_TOLERANCE = 100;
    private static final double ARM_POSITION_OFFSET = 100; // Offset for arm position adjustment
    private static final double MAX_STRAFE_ERROR = 100.0;  // Tune based on your camera/resolution
    private static final long ALIGN_TIMEOUT_MS = 3000;

    private static final double SHOULDER_LENGTH = 10.0;
    private static final double FOREARM_LENGTH = 10.0;
    //private static final double TICKS_PER_REV = 1440.0;
   private static final double SHOULDER_TICKS_PER_REV = ArmConstants.SHOULDER_TICKS_PER_REV;
 private static final double FOREARM_TICKS_PER_REV = ArmConstants.FOREARM_TICKS_PER_REV;

    private long alignStartTime = 0;

    private static final double CAMERA_FORWARD_OFFSET = 5.0; // Camera's forward offset from arm base (inches)
    private static final double CAMERA_HEIGHT_OFFSET = 8.0; // Camera's height above ground (inches)
    private static final double GROUND_HEIGHT = 2.0; // Object height (inches)
    
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
        ik = new InverseKinematics(SHOULDER_LENGTH, FOREARM_LENGTH);

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
        double width = objectDetector.getWidth();
        double strafeError = cx - TARGET_CX;

        telemetry.addData("Aligning", "CentroidX: %.1f, Width: %.1f", cx, width);
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

        if (Math.abs(strafeError) < CX_TOLERANCE && width > MIN_WIDTH_FOR_PICKUP) {
            telemetry.addData("Alignment Successful", "Ready to collect!");
            drive.stopMotors();
            currentState = RobotState.COLLECTING;
            alignStartTime = 0;
            executeCollectionSequence(width);
        } else {
            drive.drive(forwardPower, strafePower, 0);
        }
    }
// private void executeCollectionSequence(double objectWidth) {
//     double objectDistance = objectDetector.calculateDistance(objectWidth);

//     // Calculate horizontal offset from target center in pixels
//     double cx = objectDetector.getCentroidX();
//     double offsetPixels = cx - TARGET_CX;

//     // Convert pixel offset to a real-world X coordinate (you need to tune this scale factor)
//     double scaleFactor = 0.05; // meters or cm per pixel — tune as needed
//     double targetX = offsetPixels * scaleFactor;
//     double targetY = objectDistance; // forward distance from the robot

//     // Use IK to get joint angles
//     try {
//         InverseKinematics.JointAngles angles = ik.calculateJointAngles(targetX, targetY);

//         // Convert angles (degrees) to encoder ticks
//         int shoulderTicks = (int) InverseKinematics.degreesToTicks(angles.shoulderAngle, SHOULDER_TICKS_PER_REV);
//         int forearmTicks = (int) InverseKinematics.degreesToTicks(angles.forearmAngle, FOREARM_TICKS_PER_REV);

//             shoulderTicks += ArmConstants.SHOULDER_BACKLASH_COMP;
//             forearmTicks += ArmConstants.FOREARM_BACKLASH_COMP;

//         telemetry.addData("IK Shoulder Angle", angles.shoulderAngle);
//         telemetry.addData("IK Forearm Angle", angles.forearmAngle);
//         telemetry.addData("Shoulder Ticks", shoulderTicks);
//         telemetry.addData("Forearm Ticks", forearmTicks);

//         // Move arm using IK-calculated ticks
//         arm.moveArmToPosition(shoulderTicks, forearmTicks);
//         sleep(500); // wait for arm to move, tune timing

//         // Now perform gripper and other steps as before
//         arm.openGripper();
//         sleep(100);

//         arm.closeGripper();
//         sleep(100);

//         // Retract arm
//         arm.moveShoulderToPosition(500);
//         arm.rotateForearmToAngle(-500);
//         arm.openGripper();

//     } catch (IllegalArgumentException e) {
//         telemetry.addData("IK Error", e.getMessage());
//     }
// }
private void executeCollectionSequence(double objectWidth) {
    double apparentDistance = objectDetector.calculateDistance(objectWidth);
    double cx = objectDetector.getCentroidX();
    
    // Calculate real-world coordinates relative to camera
    double horizontalOffset = (cx - 160) * (apparentDistance / 500.0);
    double forwardDistance = Math.sqrt(apparentDistance*apparentDistance - horizontalOffset*horizontalOffset);
    
    // Convert to arm base coordinates
    double targetX = horizontalOffset;
    double targetY = forwardDistance + CAMERA_FORWARD_OFFSET;
    double targetZ = GROUND_HEIGHT - CAMERA_HEIGHT_OFFSET;  // Vertical offset
    
    // Use 3D position with fixed height
    double planarDistance = Math.sqrt(targetX*targetX + targetY*targetY);
    double planarAngle = Math.atan2(targetY, targetX);
    double adjustedX = planarDistance * Math.cos(planarAngle);
    double adjustedY = planarDistance * Math.sin(planarAngle);
    
    try {
        // Pass adjusted planar coordinates to IK
        InverseKinematics.JointAngles angles = ik.calculateJointAngles(adjustedX, adjustedY);
        
             InverseKinematics.JointAngles angles = ik.calculateJointAngles(targetX, targetY);

        // Convert angles (degrees) to encoder ticks
        int shoulderTicks = (int) InverseKinematics.degreesToTicks(angles.shoulderAngle, SHOULDER_TICKS_PER_REV);
        int forearmTicks = (int) InverseKinematics.degreesToTicks(angles.forearmAngle, FOREARM_TICKS_PER_REV);

            shoulderTicks += ArmConstants.SHOULDER_BACKLASH_COMP;
            forearmTicks += ArmConstants.FOREARM_BACKLASH_COMP;

        telemetry.addData("IK Shoulder Angle", angles.shoulderAngle);
        telemetry.addData("IK Forearm Angle", angles.forearmAngle);
        telemetry.addData("Shoulder Ticks", shoulderTicks);
        telemetry.addData("Forearm Ticks", forearmTicks);

        // Move arm using IK-calculated ticks
        arm.moveArmToPosition(shoulderTicks, forearmTicks);
        sleep(500); // wait for arm to move, tune timing

        // Now perform gripper and other steps as before
        arm.openGripper();
        sleep(100);

        arm.closeGripper();
        sleep(100);

        // Retract arm
        arm.moveShoulderToPosition(500);
        arm.rotateForearmToAngle(-500);
        arm.openGripper();

    } catch (Exception e) {
        telemetry.addData("IK Error", e.getMessage());
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

    private void adjustArmPositionBasedOnObject(double objectWidth) {
        double targetArmPosition = calculateArmPosition(objectWidth);
        arm.moveShoulderToPosition((int) targetArmPosition);
    }

    private double calculateArmPosition(double objectWidth) {
        double position;

        if (objectWidth < MIN_WIDTH_FOR_PICKUP) {
            position = ARM_POSITION_OFFSET;
        } else if (objectWidth < 500) {
            position = ARM_POSITION_OFFSET + 100;
        } else {
            position = ARM_POSITION_OFFSET + 200;
        }

        position = Range.clip(position, 0, 800);
        telemetry.addData("Target Arm Position", "%.1f", position);
        return position;
    }
}
