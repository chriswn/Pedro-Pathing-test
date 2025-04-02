package SubSystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystemAuto {

    public static DcMotorEx leftFront = null;
    public static DcMotorEx leftBack = null;
    public static DcMotorEx rightBack = null;
    public static DcMotorEx rightFront = null;
    
    private static final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_BASE = 16.0;    // inches between wheels
    private Telemetry telemetry;
    private static double wheelCircumference;

    public DriveSubsystemAuto (HardwareMap hardwareMap) {
        // Initialize motors
        this.telemetry = telemetry; 
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        wheelCircumference = Math.PI * WHEEL_DIAMETER;
        configureMotors();
    }

    private void configureMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Use RUN_USING_ENCODER for closed-loop control
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

   public static void forwardForDistance(double inches, Telemetry telemetry) {
        resetEncoders();
        int ticks = calculateTicks(inches);
        setTargetPositions(ticks);
        runMotorsToPosition(0.5);

        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 10) {
            telemetry.addData("Forward", "Target: %d ticks", ticks);
            telemetry.addData("FL Motor Position", leftFront.getCurrentPosition());
            telemetry.addData("FR Motor Position", rightFront.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
    }


    public void turn(int degrees, boolean clockwise) {
        double turnCircumference = Math.PI * WHEEL_BASE;
        double ticksPerDegree = (TICKS_PER_REVOLUTION / wheelCircumference) * turnCircumference / 360.0;
        int ticks = (int) (ticksPerDegree * degrees);

        if (clockwise) {
            setTargetPositionsForTurn(ticks, -ticks);
        } else {
            setTargetPositionsForTurn(-ticks, ticks);
        }

        runMotorsToPosition(0.5);

        while (motorsBusy()) {
            telemetry.addData("Turning", "Degrees: %d, Clockwise: %b", degrees, clockwise);
            telemetry.update();
        }

        stopMotors();
    }

    private static int calculateTicks(double inches) {
        double rotations = inches / wheelCircumference;
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    private static void setTargetPositions(int ticks) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + ticks);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + ticks);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + ticks);
    }

    private void setTargetPositionsForTurn(int leftTicks, int rightTicks) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + leftTicks);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + leftTicks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightTicks);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + rightTicks);
    }

    private static void runMotorsToPosition(double power) {
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    private static void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private static boolean motorsBusy() {
        return leftFront.isBusy() || leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy();
    }

    public static void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}