package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import SubSystems.Arm.ArmMovement;

@TeleOp(name = "IK Test - Tick Movement Test", group = "Test")
public class IKTicksTestOpMode extends LinearOpMode {

    private final double TICKS_PER_REVOLUTION = 560.0;

    // Initial encoder positions for shoulder and forearm
    private int initialShoulderTicks = 0;
    private int initialForearmTicks = 0;

    private final double ANGLE_INCREMENT = 5.0;  // degrees per update
    private final double SPEED_SCALING = 100.0;    // Scales joystick input for faster movement
    private final int ERROR_CORRECTION_TICKS = 500; // Additional ticks to compensate for errors

    @Override
    public void runOpMode() {
        ArmMovement arm = new ArmMovement(hardwareMap, telemetry);

        // Initializing telemetry
        telemetry.setAutoClear(false);
        telemetry.addLine("IK Test Ready. Left stick Y = Shoulder, Right stick Y = Forearm");
        telemetry.addLine("Press 'A' to Test Tick Movement.");
        telemetry.update();

        // Wait for the start of the operation
        waitForStart();

        // Capture initial encoder positions at the start
        initialShoulderTicks = arm.getShoulderPosition();
        initialForearmTicks = arm.getForearmPosition();

        while (opModeIsActive()) {
            // Adjust angles based on joystick input and scaling factor for speed
            int shoulderTicks = (int) (gamepad1.left_stick_y * ANGLE_INCREMENT * SPEED_SCALING);
            int forearmTicks = (int) (gamepad1.right_stick_y * ANGLE_INCREMENT * SPEED_SCALING);

            // Add an error correction (extra 500 ticks for tolerance)
            shoulderTicks += ERROR_CORRECTION_TICKS;
            forearmTicks += ERROR_CORRECTION_TICKS;

            // Adjust encoder values relative to initial positions
            shoulderTicks += initialShoulderTicks;
            forearmTicks += initialForearmTicks;

            // Display telemetry for joint control
            telemetry.clear();
            telemetry.addLine("Direct Joint Control (Faster):");
            telemetry.addData("Target Shoulder Ticks", shoulderTicks);
            telemetry.addData("Target Forearm Ticks", forearmTicks);

            // Display current position of the motors
            telemetry.addLine("Current Encoder Ticks:");
            telemetry.addData("Current Shoulder Ticks", arm.getShoulderPosition());
            telemetry.addData("Current Forearm Ticks", arm.getForearmPosition());

            // Test tick movement when 'A' is pressed
            if (gamepad1.a) {
                long startTime = System.currentTimeMillis();

                // Move the arm to target ticks based on joystick input
                arm.moveArmToPosition(shoulderTicks, forearmTicks);

                long endTime = System.currentTimeMillis();
                long elapsedTime = endTime - startTime;

                telemetry.addLine("Tick Movement Test:");
                telemetry.addData("Time Taken (ms)", elapsedTime);
                telemetry.addData("Ticks (Shoulder)", shoulderTicks);
                telemetry.addData("Ticks (Forearm)", forearmTicks);
            }

            // Update telemetry
            telemetry.update();

            sleep(50); // Reduced sleep for faster loop execution
        }
    }
}
