package AUTO;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Point;

import SubSystems.Drive.DriveSubsystemAuto;
import SubSystems.Vision.ObjectDetectionPipeline;
import SubSystems.Arm.ArmMovement;

@Autonomous(name = " testArm Pickup", group = "Competition")
public class testArmPickup extends LinearOpMode {
    // Subsystems
    private DriveSubsystemAuto drive;
    private ArmMovement arm;

    // Vision
    private ObjectDetectionPipeline objectDetector;
    private VisionPortal visionPortal;

    // Constants
    private static final double STRAFE_KP = 0.015;
    private static final double FORWARD_KP = 0.005;
    private static final int TARGET_CX = 160;
    private static final double CX_TOLERANCE = 15;
    private static final double ARM_POSITION_OFFSET = 100;
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

        while (opModeIsActive()) {
            stateMachineLoop();
            updateTelemetry();
            sleep(15);
        }

        visionPortal.close();
    }

    private void initializeSystems() {
        drive = new DriveSubsystemAuto(hardwareMap, telemetry);
        arm = new ArmMovement(hardwareMap, telemetry);

        objectDetector = new ObjectDetectionPipeline();
        objectDetector.setDetectionMode(ObjectDetectionPipeline.DetectionMode.RED_YELLOW);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor((VisionProcessor) objectDetector)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        arm.moveShoulderToPosition(100);
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
                if (objectDetector.isObjectDetected()) {
                    currentState = RobotState.ALIGNING;
                    alignStartTime = System.currentTimeMillis();
                } else {
                    drive.drive(0, 0, 0.2);
                }
                break;

            case ALIGNING:
                if (!objectDetector.isObjectDetected() ||
                        (System.currentTimeMillis() - alignStartTime > ALIGN_TIMEOUT_MS)) {
                    currentState = RobotState.SEARCHING;
                    break;
                }

                Point center = objectDetector.getObjectCenter();
                double strafeError = center.x - TARGET_CX;
                double forwardError = 240 - objectDetector.getObjectWidth();

                double strafe = Range.clip(strafeError * STRAFE_KP, -0.3, 0.3);
                double forward = Range.clip(forwardError * FORWARD_KP, -0.2, 0.2);

                if (Math.abs(strafeError) < CX_TOLERANCE) {
                    currentState = RobotState.COLLECTING;
                } else {
                    drive.drive(forward, strafe, 0);
                }
                break;

            case COLLECTING:
                arm.closeGripper();
                sleep(500);
                currentState = RobotState.RETRACTING;
                break;

            case RETRACTING:
                arm.moveShoulderToPosition((int) ARM_POSITION_OFFSET);
                sleep(500);
                currentState = RobotState.COMPLETE;
                break;

            case COMPLETE:
                drive.stopMotors();
                break;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Detected", objectDetector.isObjectDetected());
        telemetry.addData("Color", objectDetector.getDetectedColor());
        telemetry.addData("Center", objectDetector.getObjectCenter());
        telemetry.addData("Width", objectDetector.getObjectWidth());
        telemetry.addData("Distance", objectDetector.estimateDistance());
        telemetry.update();
    }
}
