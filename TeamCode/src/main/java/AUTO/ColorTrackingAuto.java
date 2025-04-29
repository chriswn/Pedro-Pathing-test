package AUTO;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvPipeline;

import SubSystems.Drive.DriveSubsystemAuto;
import SubSystems.Vision.TurboObjectDetectionPipeline;

@Autonomous(name = "Color Tracking Auto (with DriveSubsystem)")
public class ColorTrackingAuto extends LinearOpMode {

    private TurboObjectDetectionPipeline objectDetector;
    private VisionPortal visionPortal;

    private DriveSubsystemAuto drive;

    private final double STRAFE_Kp = 0.005;
    private final double FORWARD_Kp = 0.002;

    private boolean togglePressed = false;
    private TurboObjectDetectionPipeline.DetectionMode currentMode = TurboObjectDetectionPipeline.DetectionMode.RED_YELLOW;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystemAuto(hardwareMap, telemetry);
        initVision();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                handleModeSwitch();

                if (objectDetector.isObjectDetected()) {
                    moveToObject();
                } else {
                    drive.stopMotors();
                    telemetry.addData("Detection", "No object detected");
                }

                telemetry.addData("Detected Color", objectDetector.getDetectedColor());
                telemetry.addData("Object CX", objectDetector.getCentroidX());
                telemetry.addData("Object Width", objectDetector.getWidth());
                telemetry.update();
                sleep(50);
            }
        }
        visionPortal.close();
    }

    private void initVision() {
        objectDetector = new TurboObjectDetectionPipeline();

        // Initialize VisionPortal with the webcam and the TurboObjectDetectionPipeline
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(objectDetector)  // Add your object detection pipeline here
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

    private void moveToObject() {
        int imageCenterX = 320;  // Half of the camera width (640px)
        double objectX = objectDetector.getCentroidX();
        double objectWidth = objectDetector.getWidth();

        // Calculate the error from the center of the image
        double errorX = objectX - imageCenterX;

        // Simple PID-like control (not exact, but works for this example)
        double strafePower = errorX * STRAFE_Kp;
        double forwardPower = (300 - objectWidth) * FORWARD_Kp;

        // Clamp the power to prevent over-driving
        strafePower = Math.max(-0.4, Math.min(0.4, strafePower));
        forwardPower = Math.max(-0.4, Math.min(0.4, forwardPower));

        // Command the robot to drive towards the object
        drive.drive(forwardPower, strafePower, 0);
    }

    private void handleModeSwitch() {
        // Switch between RED_YELLOW and BLUE_YELLOW modes when A is pressed
        if (gamepad1.a && !togglePressed) {
            togglePressed = true;

            currentMode = (currentMode == TurboObjectDetectionPipeline.DetectionMode.RED_YELLOW)
                    ? TurboObjectDetectionPipeline.DetectionMode.BLUE_YELLOW
                    : TurboObjectDetectionPipeline.DetectionMode.RED_YELLOW;

            objectDetector.setDetectionMode(currentMode);  // Update detection mode
            telemetry.addData("Switched Detection Mode", currentMode);
        }
        if (!gamepad1.a) {
            togglePressed = false;
        }
    }
}
