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

@Autonomous(name = "Full Auto Pickup", group = "Competition")
public class AutoPickup extends LinearOpMode {
    // Subsystems
    private DriveSubsystemAuto drive;
    private ArmMovement arm;
    
    // Vision
    private TurboObjectDetectionPipeline objectDetector;
    private VisionPortal visionPortal;
    
    // Constants
    private static final double STRAFE_KP = 0.008;
    private static final double FORWARD_KP = 0.003;
    private static final int TARGET_CX = 320;
    private static final int MIN_WIDTH_FOR_PICKUP = 280;
    private static final int CX_TOLERANCE = 35;
    
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
                sleep(20);
            }
        }
        visionPortal.close();
    }

    private void initializeSystems() {
        // Initialize subsystems
        drive = new DriveSubsystemAuto(hardwareMap, telemetry);
        arm = new ArmMovement(hardwareMap, telemetry);
        
        // Initialize vision
        objectDetector = new TurboObjectDetectionPipeline();
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(objectDetector)
            .setCameraResolution(new Size(640, 480))
            .build();
            
        // Arm initialization sequence
        arm.moveShoulderToPosition(100); // Engage encoders
        sleep(500);
        arm.stopMotors();
        arm.closeGripper(); // Start with closed gripper
        
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
            // Perform search pattern
            drive.drive(0, 0, 0.3); // Rotate slowly
        }
    }

    private void handleAlignState() {
        if (!objectDetector.isObjectDetected()) {
            currentState = RobotState.SEARCHING;
            return;
        }
        
        double cx = objectDetector.getCentroidX();
        double width = objectDetector.getWidth();
        
        // Calculate alignment errors
        double strafeError = cx - TARGET_CX;
        double forwardError = MIN_WIDTH_FOR_PICKUP - width;
        
        // Calculate motor powers
        double strafePower = strafeError * STRAFE_KP;
        double forwardPower = forwardError * FORWARD_KP;
        
        // Clamp values
        strafePower = Range.clip(strafePower, -0.5, 0.5);
        forwardPower = Range.clip(forwardPower, -0.4, 0.4);
        
        // Check if ready for collection
        if (Math.abs(strafeError) < CX_TOLERANCE && width > MIN_WIDTH_FOR_PICKUP) {
            drive.stopMotors();
            currentState = RobotState.COLLECTING;
            executeCollectionSequence();
        } else {
            drive.drive(forwardPower, strafePower, 0);
        }
    }

    private void executeCollectionSequence() {
        // Lower arm to collect position
        arm.moveShoulderToPosition(-350);
        sleep(1000);
        
        // Position forearm
        arm.rotateForearmToAngle(220);
        sleep(800);
        
        // Final approach
        drive.drive(0.15, 0, 0);
        sleep(300);
        drive.stopMotors();
        
        // Open and close gripper
        arm.openGripper();
        sleep(500);
        arm.closeGripper();
        sleep(800);
        
        // Lift arm
        arm.moveShoulderToPosition(400);
        arm.rotateForearmToAngle(-180);
        sleep(1000);
        
        currentState = RobotState.RETRACTING;
    }

    private void handleCollectState() {
        executeCollectionSequence();
    }

    private void handleRetractState() {
        // Back away from collection area
        drive.drive(-0.3, 0, 0);
        sleep(1000);
        drive.stopMotors();
        currentState = RobotState.COMPLETE;
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState.toString());
        telemetry.addData("Object Detected", objectDetector.isObjectDetected());
        
        if (objectDetector.isObjectDetected()) {
            telemetry.addData("Object CX", "%.1f", objectDetector.getCentroidX());
            telemetry.addData("Object Width", "%.1f", objectDetector.getWidth());
            telemetry.addData("Detected Color", objectDetector.getDetectedColor());
        }
        
        telemetry.addData("Shoulder Pos", arm.getShoulderPosition());
        telemetry.addData("Forearm Pos", arm.getForearmPosition());
        telemetry.update();
    }
}