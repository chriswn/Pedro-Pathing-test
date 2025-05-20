package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import SubSystems.Drive.DriveSubsystemAuto;

@Autonomous(name = "Drive Test OpMode", group = "Test")
public class DriveTestOpMode extends LinearOpMode {

    private DriveSubsystemAuto drive;

    @Override
    public void runOpMode() {
        // Initialize DriveSubsystemAuto
        drive = new DriveSubsystemAuto(hardwareMap, telemetry);

        waitForStart();

        if (opModeIsActive()) {
            // Drive forward for 2 seconds
            drive.drive(0.5, 0, 0);  // Move forward
            sleep(2000000000);
            drive.stopMotors();

//            // Drive sideways for 2 seconds
//            drive.drive(0, 0.5, 0);  // Move sideways
//            sleep(2000);
//            drive.stopMotors();
//
//            // Turn for 2 seconds
//            drive.drive(0, 0, 0.5);  // Turn
//            sleep(2000);
//            drive.stopMotors();

            // Test each motor individually using the custom methods in DriveSubsystemAuto
            testMotor("Left Front");
            testMotor("Left Back");
            testMotor("Right Front");
            testMotor("Right Back");
        }
    }

    // Test individual motors by using the DriveSubsystemAuto's power control methods
    private void testMotor(String motorName) {
        // Simple way to test motor: Set power to 1, then -1
        telemetry.addData(motorName + " Test", "Power: 1");
        telemetry.update();
        drive.drive(1, 0, 0);  // Move forward
        sleep(1000);
        drive.stopMotors();

        telemetry.addData(motorName + " Test", "Power: -1");
        telemetry.update();
        drive.drive(-1, 0, 0);  // Move backward
        sleep(1000);
        drive.stopMotors();

        sleep(500);  // Wait between tests
    }
}
