package SubSystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmMovement {

    // Declare motors and servos
    private DcMotor forearm = null;
    private DcMotor shoulder = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    // Constants
    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double MOTOR_POWER = 0.5;
    private static final double SERVO_OPEN_POSITION = 1.0;
    private static final double SERVO_CLOSED_POSITION = 0.0;

    private Telemetry telemetry;

    // Constructor to initialize hardware
    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        forearm = hardwareMap.get(DcMotor.class, "forearm");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        forearm.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);

        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Expose motor positions
    public int getShoulderPosition() {
        return shoulder.getCurrentPosition();
    }

    public int getForearmPosition() {
        return forearm.getCurrentPosition();
    }

    // Move both arm components
    public void moveArmToPosition(int shoulderTicks, int forearmTicks) {
        shoulder.setTargetPosition(shoulder.getCurrentPosition() + shoulderTicks);
        forearm.setTargetPosition(forearm.getCurrentPosition() + forearmTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shoulder.setPower(MOTOR_POWER);
        forearm.setPower(MOTOR_POWER);

        ElapsedTime runtime = new ElapsedTime();
        while ((shoulder.isBusy() || forearm.isBusy()) && runtime.seconds() < 5) {
            telemetry.addData("Shoulder Position", shoulder.getCurrentPosition());
            telemetry.addData("Forearm Position", forearm.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }

    // Move only shoulder
    public void moveShoulderToPosition(int ticks) {
        moveArmToPosition(ticks, 0);
    }

    // Move only forearm
    public void rotateForearmToAngle(int ticks) {
        moveArmToPosition(0, ticks);
    }

    // New method: move shoulder with timeout
    public boolean moveShoulderToPosition(int ticks, double timeoutS) {
        ElapsedTime timer = new ElapsedTime();
        shoulder.setTargetPosition(ticks);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(MOTOR_POWER);

        while (shoulder.isBusy() && timer.seconds() < timeoutS) {
            // Optionally add telemetry here
        }

        shoulder.setPower(0);
        return !shoulder.isBusy(); // true if successful
    }

    public void openGripper() {
        leftClaw.setPosition(SERVO_OPEN_POSITION);
        rightClaw.setPosition(SERVO_OPEN_POSITION);
        telemetry.addData("Gripper", "Opened");
        telemetry.update();
    }

    public void closeGripper() {
        leftClaw.setPosition(SERVO_CLOSED_POSITION);
        rightClaw.setPosition(SERVO_CLOSED_POSITION);
        telemetry.addData("Gripper", "Closed");
        telemetry.update();
    }

    public void stopMotors() {
        shoulder.setPower(0);
        forearm.setPower(0);
        telemetry.addData("Motors", "Stopped");
        telemetry.update();
    }
}
