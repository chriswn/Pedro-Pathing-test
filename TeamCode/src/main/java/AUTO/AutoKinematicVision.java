package AUTO;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import SubSystems.Arm.ArmMovement;
import SubSystems.Vision.TurboObjectDetectionPipeline;
import SubSystems.InverseKinematics;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
@Autonomous(name = "AutoKinematicVision", group = "Auto")
public class AutoKinematicVision extends LinearOpMode {
    private ArmMovement arm;
    private TurboObjectDetectionPipeline pipeline;
    private VisionPortal visionPortal;
    private InverseKinematics ik;

    private static final double SHOULDER_LENGTH = 10.0;
    private static final double FOREARM_LENGTH = 10.0;
    private static final double TICKS_PER_REV = 1440.0;

    @Override
    public void runOpMode() {
        ik = new InverseKinematics(SHOULDER_LENGTH, FOREARM_LENGTH);

        // Initialize arm subsystem
        arm = new ArmMovement(hardwareMap, telemetry);

        // Vision setup
        pipeline = new TurboObjectDetectionPipeline();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(pipeline)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (pipeline.isObjectDetected()) {
                double distance = pipeline.calculateDistance(pipeline.getWidth());
                double offsetX = pipeline.cX - 160;
                double scaleFactor = 0.05;
                double targetX = offsetX * scaleFactor;
                double targetY = distance;

                try {
                    InverseKinematics.JointAngles angles = ik.calculateJointAngles(targetX, targetY);
                    double shoulderTicks = InverseKinematics.degreesToTicks(angles.shoulderAngle, TICKS_PER_REV);
                    double forearmTicks = InverseKinematics.degreesToTicks(angles.forearmAngle, TICKS_PER_REV);

                    telemetry.addData("Target X", targetX);
                    telemetry.addData("Target Y", targetY);
                    telemetry.addData("Shoulder Angle (deg)", angles.shoulderAngle);
                    telemetry.addData("Forearm Angle (deg)", angles.forearmAngle);
                    telemetry.addData("Shoulder Ticks", shoulderTicks);
                    telemetry.addData("Forearm Ticks", forearmTicks);

                    arm.moveArmToPosition((int) shoulderTicks, (int) forearmTicks);

                } catch (IllegalArgumentException e) {
                    telemetry.addData("IK Error", e.getMessage());
                }
            } else {
                telemetry.addLine("No object detected.");
            }
            telemetry.update();
            sleep(100);
        }
    }
}
