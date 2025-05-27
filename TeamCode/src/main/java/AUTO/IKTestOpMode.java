package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import SubSystems.InverseKinematics;
import SubSystems.Arm.ArmMovement;

@TeleOp(name = "IK Test - Faster Joint Control", group = "Test")
public class IKTestOpMode extends LinearOpMode {

    private final double SHOULDER_LENGTH = 20;  // Shoulder length in cm (or inches, adjust as needed)
    private final double FOREARM_LENGTH = 20;   // Forearm length in cm (or inches, adjust as needed)
    private final double TICKS_PER_REVOLUTION = 560.0;  // Number of ticks per full revolution (adjust as needed)

    // Initial angles in degrees
    private double shoulderAngle = 0;
    private double forearmAngle = 0;

    // Initial encoder positions for shoulder and forearm
    private int initialShoulderTicks = 0;
    private int initialForearmTicks = 0;

    private final double ANGLE_INCREMENT = 2.0;  // degrees per update
    private final double SPEED_SCALING = 50.0;    // Scales joystick input for faster movement
    private final int ERROR_CORRECTION_TICKS = 10;  // Additional ticks to compensate for errors

    // Calculate the ticks per degree for a more accurate conversion
    private final double TICKS_PER_DEGREE = 8000.0 / 360.0;  // Adjust based on your system (e.g., 8000 ticks for 360°)

    @Override
    public void runOpMode() {
        // Initialize Inverse Kinematics and ArmMovement subsystems
        InverseKinematics ik = new InverseKinematics(SHOULDER_LENGTH, FOREARM_LENGTH);
        ArmMovement arm = new ArmMovement(hardwareMap, telemetry);

        // Initializing telemetry
        telemetry.setAutoClear(false);
        telemetry.addLine("IK Test Ready. Left stick Y = Shoulder, Right stick Y = Forearm");
        telemetry.update();

        // Capture initial encoder positions at the start
        initialShoulderTicks = arm.getShoulderPosition();
        initialForearmTicks = arm.getForearmPosition();

        // Wait for the start of the operation
        waitForStart();

        while (opModeIsActive()) {
            // Adjust angles based on joystick input and scaling factor for speed
            shoulderAngle -= gamepad1.left_stick_y * ANGLE_INCREMENT * SPEED_SCALING;
            forearmAngle -= gamepad1.right_stick_y * ANGLE_INCREMENT * SPEED_SCALING;

            // Clamp angles to safe limits (optional, adjust as needed)
            shoulderAngle = Math.max(0, Math.min(150, shoulderAngle));
            forearmAngle = Math.max(0, Math.min(360, forearmAngle));

            // Calculate ticks from angles (adjust based on system)
            int shoulderTicks = (int) (shoulderAngle * TICKS_PER_DEGREE);  // Convert shoulder angle to ticks
            int forearmTicks = (int) (forearmAngle * TICKS_PER_DEGREE);    // Convert forearm angle to ticks

            // Adjust for mechanical issues (play in gears, reverse direction, etc.)
            shoulderTicks = -shoulderTicks;  // Invert shoulder ticks (fix for reverse direction)
            forearmTicks += 150;  // Add compensation for forearm resting position (play in gears)
            shoulderTicks += 50;  // Add compensation for shoulder play (adjust as necessary)

            // Add error correction (extra ticks for tolerance)
            shoulderTicks += ERROR_CORRECTION_TICKS;
            forearmTicks += ERROR_CORRECTION_TICKS;

            // Adjust encoder values relative to initial positions
            shoulderTicks -= initialShoulderTicks;
            forearmTicks -= initialForearmTicks;

            // Display telemetry for joint control
            telemetry.clear();
            telemetry.addLine("Direct Joint Control (Faster):");
            telemetry.addData("Shoulder Angle", "%.2f deg", shoulderAngle);
            telemetry.addData("Forearm Angle", "%.2f deg", forearmAngle);

            telemetry.addLine("Encoder Ticks (with error correction):");
            telemetry.addData("Shoulder", shoulderTicks);
            telemetry.addData("Forearm", forearmTicks);

            // Move the arm using the subsystem
            arm.moveArmToPosition(shoulderTicks, forearmTicks);

            // Update telemetry
            telemetry.update();

            // Sleep for a short period to allow for smoother operation
            sleep(50); // Reduced sleep for faster loop execution
        }
    }
}
