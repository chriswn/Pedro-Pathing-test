package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import SubSystems.InverseKinematics;
import SubSystems.InverseKinematics.JointAngles;

@TeleOp(name = "IK Test OpMode", group = "Test")
public class IKTestOpMode extends LinearOpMode {

    // Adjustable target positions using gamepad
    private double targetX = 10; // in cm or inches (depends on your units)
    private double targetY = 10;

    // Constants for your robot's arm
    private final double SHOULDER_LENGTH = 20; // Set this to your real value
    private final double FOREARM_LENGTH = 20;  // Set this to your real value
    private final double TICKS_PER_REVOLUTION = 1440; // Adjust to match your encoder

    @Override
    public void runOpMode() {
        InverseKinematics ik = new InverseKinematics(SHOULDER_LENGTH, FOREARM_LENGTH);

        telemetry.setAutoClear(false);
        telemetry.addLine("IK Test Ready. Use left stick to move target (X/Y).");
        telemetry.addLine("Press start to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust targetX and targetY using the left stick
            targetX += gamepad1.left_stick_x;
            targetY -= gamepad1.left_stick_y; // Invert Y for intuitive up/down

            try {
                JointAngles angles = ik.calculateJointAngles(targetX, targetY);

                double shoulderTicks = InverseKinematics.degreesToTicks(angles.shoulderAngle, TICKS_PER_REVOLUTION);
                double forearmTicks = InverseKinematics.degreesToTicks(angles.forearmAngle, TICKS_PER_REVOLUTION);

                telemetry.clear();
                telemetry.addLine("Target Position:");
                telemetry.addData("X", "%.2f", targetX);
                telemetry.addData("Y", "%.2f", targetY);
                telemetry.addLine();

                telemetry.addLine("Calculated Joint Angles:");
                telemetry.addData("Shoulder Angle (deg)", "%.2f", angles.shoulderAngle);
                telemetry.addData("Forearm Angle (deg)", "%.2f", angles.forearmAngle);
                telemetry.addLine();

                telemetry.addLine("Corresponding Encoder Ticks:");
                telemetry.addData("Shoulder Ticks", "%.2f", shoulderTicks);
                telemetry.addData("Forearm Ticks", "%.2f", forearmTicks);
                telemetry.update();
            } catch (IllegalArgumentException e) {
                telemetry.clear();
                telemetry.addLine("ERROR: " + e.getMessage());
                telemetry.addData("X", "%.2f", targetX);
                telemetry.addData("Y", "%.2f", targetY);
                telemetry.update();
            }

            sleep(100); // Slow down updates
        }
    }
}
