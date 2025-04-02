package SubSystems.Vision;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Comparator;

import SubSystems.Drive.DriveSubsystem;

public class AprilTagNavigator {

    private DriveSubsystem driveSubsystem;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    // Robot motors
    public final  DcMotor leftFront, leftBack, rightBack, rightFront;

    // PID tuning values (adjust as needed)
    private final double STRAFE_Kp = 0.015; // Lower for smoother movement
    private final double FORWARD_Kp = 0.015;
    private final double TURN_Kp = 0.008;

    private final double DEADBAND = 0.5; // Ignore small errors to prevent jitter

    public AprilTagNavigator(DriveSubsystem driveSubsystem, HardwareMap hardwareMap, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.telemetry = telemetry;

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        setMotorModes();

        // Initialize AprilTag vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void setMotorModes() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public AprilTagDetection getClosestTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return null;

        // Find the closest tag based on Z distance
        return detections.stream()
                .min(Comparator.comparingDouble(tag -> Math.abs(tag.ftcPose.z)))
                .orElse(null);
    }

    public void alignToTag(AprilTagDetection tag) {
        if (tag == null) {
            stopRobot();
            return;
        }

        double xError = tag.ftcPose.x;  // Left/Right
        double yError = tag.ftcPose.y;  // Forward/Backward
        double yawError = tag.ftcPose.yaw;  // Rotation

        telemetry.addData("Aligning to Tag", "X: %.2f, Y: %.2f, Yaw: %.2f", xError, yError, yawError);
        telemetry.update();

        // Apply deadband filter (ignore small errors)
        if (Math.abs(xError) < DEADBAND) xError = 0;
        if (Math.abs(yError) < DEADBAND) yError = 0;
        if (Math.abs(yawError) < DEADBAND) yawError = 0;

        double strafePower = Range.clip(xError * STRAFE_Kp, -0.4, 0.4);
        double forwardPower = Range.clip(yError * FORWARD_Kp, -0.4, 0.4);
        double turnPower = Range.clip(yawError * TURN_Kp, -0.3, 0.3);

        moveRobot(forwardPower, strafePower, turnPower);
    }

    private void moveRobot(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        leftFront.setPower(Range.clip(fl, -1, 1));
        rightFront.setPower(Range.clip(fr, -1, 1));
        leftBack.setPower(Range.clip(bl, -1, 1));
        rightBack.setPower(Range.clip(br, -1, 1));
    }

    public void stopRobot() {
        moveRobot(0, 0, 0);
    }

    public void closeVision() {
        visionPortal.close();
    }
}
