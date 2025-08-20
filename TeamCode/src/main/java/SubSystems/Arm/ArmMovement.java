package SubSystems.Arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmMovement {
    // Declare motors and servos
    private DcMotor forearm = null;
    private DcMotor shoulder = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    // Constants
    private static final double MOTOR_POWER = 1.0;
    private static final double SERVO_OPEN_POSITION = 0.3;
    private static final double SERVO_CLOSED_POSITION = 1.0;
    
    // Gravity compensation parameters
    private static final double GRAVITY_COMP_FACTOR = 0.3; // Tune this (0.0-1.0)
    private static final double PAYLOAD_WEIGHT = 0.5; // lbs (estimate)

    private Telemetry telemetry;

    public ArmMovement(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize hardware
        forearm = hardwareMap.get(DcMotor.class, "forearm");
        shoulder = hardwareMap.get(DcMotor.class, "shoulder");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set motor directions
        forearm.setDirection(DcMotor.Direction.REVERSE);
        shoulder.setDirection(DcMotor.Direction.REVERSE);

        // Configure encoders
        resetEncoders();
    }

    public void resetEncoders() {
        forearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Get current positions
    public int getShoulderPosition() {
        return shoulder.getCurrentPosition();
    }

    public int getForearmPosition() {
        return forearm.getCurrentPosition();
    }

    // Main movement method
    public void moveArmToPosition(int shoulderTargetTicks, int forearmTargetTicks) {
        shoulder.setTargetPosition(shoulderTargetTicks);
        forearm.setTargetPosition(forearmTargetTicks);

        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        forearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Apply gravity compensation to shoulder
        double shoulderPower = calculateShoulderPower(shoulderTargetTicks);
        shoulder.setPower(1);
        forearm.setPower(MOTOR_POWER);

        ElapsedTime runtime = new ElapsedTime();
        while ((shoulder.isBusy() || forearm.isBusy()) && runtime.seconds() < 5) {
            telemetry.addData("Shoulder Position", shoulder.getCurrentPosition());
            telemetry.addData("Forearm Position", forearm.getCurrentPosition());
            telemetry.update();
        }
        
        // Maintain position against gravity
        shoulder.setPower(shoulderPower * 0.3); // Reduced holding power
        forearm.setPower(0.1);
    }

    // Gravity compensation calculation
    private double calculateShoulderPower(int shoulderTicks) {
        // Convert ticks to appoxirmate angle 
        // 0° = horizontal, 90° = vertical
        double angle = (shoulderTicks / 10.0) % 360; // Tune conversion factor
        
        // Simple gravity compensation model
        double compensation = GRAVITY_COMP_FACTOR * Math.sin(Math.toRadians(angle)) * PAYLOAD_WEIGHT;
        
        // Ensure power is within valid range
        return Range.clip(MOTOR_POWER + compensation, 0.1, 1.0);
    }

    // Move only shoulder
    public void moveShoulderToPosition(int ticks) {
        int currentForearm = forearm.getCurrentPosition();
        moveArmToPosition(ticks, currentForearm);
    }

    // Move only forearm
    public void rotateForearmToAngle(int ticks) {
        int currentShoulder = shoulder.getCurrentPosition();
        moveArmToPosition(currentShoulder, ticks);
    }

    // Gripper control - maintain your tested logic
    public void openGripper() {
        leftClaw.setPosition(SERVO_OPEN_POSITION);
        rightClaw.setPosition(SERVO_CLOSED_POSITION);
        telemetry.addData("Gripper", "Opened");
        telemetry.update();
    }
    
    public void closeGripper() {
        leftClaw.setPosition(SERVO_CLOSED_POSITION);
        rightClaw.setPosition(SERVO_OPEN_POSITION);
        telemetry.addData("Gripper", "Closed");
        telemetry.update();
    }
    
    public void Gripper() {
        telemetry.addData("Left Claw", leftClaw.getPosition());
        telemetry.addData("Right Claw", rightClaw.getPosition());
        telemetry.update();
    }

    public void stopMotors() {
        shoulder.setPower(0);
        forearm.setPower(0);
        telemetry.addData("Motors", "Stopped");
        telemetry.update();
    }
    
    //  gravity-affected movements
    public void moveShoulderWithCompensation(int ticks) {
        double power = calculateShoulderPower(ticks);
        shoulder.setTargetPosition(ticks);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        forearm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shoulder.setPower(power);
        
        ElapsedTime timer = new ElapsedTime();
        while (shoulder.isBusy() && timer.seconds() < 3) {
            // Update telemetry if needed
        }
        shoulder.setPower(power * 0.3); // Reduced holding power
    }
}