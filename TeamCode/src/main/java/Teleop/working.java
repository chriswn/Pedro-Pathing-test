package Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import SubSystems.Arm.ArmMovement;
import SubSystems.Drive.DriveSubsystemAuto;

@TeleOp(name = "working", group = "Teleop")
public class working extends LinearOpMode {

    private DriveSubsystemAuto drive;
    private ArmMovement arm;

    //  positions (in encoder ticks)
    private final int SHOULDER_UP = 500;
    private final int SHOULDER_DOWN = 0;
    private final int FOREARM_EXTEND = 400;
    private final int FOREARM_RETRACT = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize subsystems
        drive = new DriveSubsystemAuto(hardwareMap,telemetry);
        arm = new ArmMovement(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ----- Drive -----
            double forward = -gamepad1.left_stick_y;  // Forward/backward
            double strafe = gamepad1.left_stick_x;    // Left/right
            double turn = gamepad1.right_stick_x;     // Rotation

            drive.drive(forward, strafe, turn);

            // ----- Arm control -----
            if (gamepad2.dpad_up) {
                arm.moveShoulderToPosition(SHOULDER_UP);
            } else if (gamepad2.dpad_down) {
                arm.moveShoulderToPosition(SHOULDER_DOWN);
            }

            if (gamepad2.dpad_right) {
                arm.rotateForearmToAngle(FOREARM_EXTEND);
            } else if (gamepad2.dpad_left) {
                arm.rotateForearmToAngle(FOREARM_RETRACT);
            }

            // ----- Gripper -----
            if (gamepad2.a) {
                arm.openGripper();
            } else if (gamepad2.b) {
                arm.closeGripper();
            }

            // Show live telemetry
            telemetry.addData("Shoulder Position", arm.getShoulderPosition());
            telemetry.addData("Forearm Position", arm.getForearmPosition());
            telemetry.update();
        }

        // Stop all motors after opMode ends
        drive.stop();
        arm.stopMotors();
    }
}
