package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Constants.ArmConstants;
import SubSystems.InverseKinematics;
import SubSystems.Arm.ArmMovement;

@TeleOp(name = "IK Test - Faster Joint Control", group = "Test")
public class IKTestOpMode extends LinearOpMode {

    private final double SHOULDER_LENGTH = 20.0;  // in cm or inches
    private final double FOREARM_LENGTH = 20.0;   // in cm or inches

    // Initial angles in degrees
    private double shoulderAngle = 0;
    private double forearmAngle = 0;

    // Initial encoder positions for shoulder and forearm
    private int initialShoulderTicks = 0;
    private int initialForearmTicks = 0;

    private final double ANGLE_INCREMENT = 2.0;     // degrees per update
    private final double SPEED_SCALING = 50.0;      // Scales joystick input

    @Override
    public void runOpMode() {
        // Initialize Inverse Kinematics and Arm subsystem
        InverseKinematics ik = new InverseKinematics(SHOULDER_LENGTH, FOREARM_LENGTH);
        ArmMovement arm = new ArmMovement(hardwareMap, telemetry);

        telemetry.setAutoClear(false);
        telemetry.addLine("IK Test Ready. Left stick Y = Shoulder, Right stick Y = Forearm");
        telemetry.update();

        //  initial encoder positions
        initialShoulderTicks = arm.getShoulderPosition();
        initialForearmTicks = arm.getForearmPosition();

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Adjust angles based on joystick input
            shoulderAngle -= gamepad1.left_stick_y * ANGLE_INCREMENT * SPEED_SCALING;
            forearmAngle -= gamepad1.right_stick_y * ANGLE_INCREMENT * SPEED_SCALING;

            // Clamp angles
            shoulderAngle = Math.max(0, Math.min(150, shoulderAngle));
            forearmAngle = Math.max(0, Math.min(360, forearmAngle));

            // Convert angles to ticks using constants
            int shoulderTicks = (int) (shoulderAngle * ArmConstants.SHOULDER_TICKS_PER_DEGREE);
            int forearmTicks = (int) (forearmAngle * ArmConstants.FOREARM_TICKS_PER_DEGREE);

            // Apply hardware compensation (backlash, offset, direction)
            shoulderTicks = -shoulderTicks + ArmConstants.SHOULDER_BACKLASH_COMP;
            forearmTicks = forearmTicks + ArmConstants.FOREARM_BACKLASH_COMP;

            // Adjust for initial encoder positions
            shoulderTicks -= initialShoulderTicks;
            forearmTicks -= initialForearmTicks;

            // Telemetry display
            telemetry.clear();
            telemetry.addLine("Direct Joint Control (Faster):");
            telemetry.addData("Shoulder Angle", "%.2f deg", shoulderAngle);
            telemetry.addData("Forearm Angle", "%.2f deg", forearmAngle);

            telemetry.addLine("Encoder Ticks (adjusted):");
            telemetry.addData("Shoulder", shoulderTicks);
            telemetry.addData("Forearm", forearmTicks);

            // Move arm
            arm.moveArmToPosition(shoulderTicks, forearmTicks);

            telemetry.update();
            sleep(50); // smooth control loop
        }
    }
}
