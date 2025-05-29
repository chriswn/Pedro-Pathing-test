package SubSystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystemAuto {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;
    private DcMotorEx rightFront;
    
    private final double TICKS_PER_REVOLUTION = 560.0;
    private static final double WHEEL_DIAMETER = 4.0; // inches
    private static final double WHEEL_BASE = 16.0;    // inches between wheels
    private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private Telemetry telemetry;

    public DriveSubsystemAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        configureMotors();
    }

    private void configureMotors() {
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);


        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;
        telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", fl, fr, bl, br);
        telemetry.update();
        if (strafe == 0 && turn == 0) {
            fl = fr = bl = br = forward;  // All motors will run at the same speed.
        }

        setMotorPowers(fl, fr, bl, br);
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void forwardForDistance(double inches) {
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
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn(int degrees, boolean clockwise) {
        double turnCircumference = Math.PI * WHEEL_BASE;
        double ticksPerDegree = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE) * turnCircumference / 360.0;
        int ticks = (int) (ticksPerDegree * degrees);

        resetEncoders();
        if (clockwise) {
            setTargetPositionsForTurn(ticks, -ticks);
        } else {
            setTargetPositionsForTurn(-ticks, ticks);
        }

        runMotorsToPosition(0.5);

        ElapsedTime runtime = new ElapsedTime();
        while (motorsBusy() && runtime.seconds() < 10) {
            telemetry.addData("Turning", "Degrees: %d, Clockwise: %b", degrees, clockwise);
            telemetry.update();
        }

        stopMotors();
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private int calculateTicks(double inches) {
        double rotations = inches / WHEEL_CIRCUMFERENCE;
        return (int) (rotations * TICKS_PER_REVOLUTION);
    }

    private void setTargetPositions(int ticks) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + ticks);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + ticks);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + ticks);
    }

    private void setTargetPositionsForTurn(int leftTicks, int rightTicks) {
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + leftTicks);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + leftTicks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + rightTicks);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + rightTicks);
    }

    private void runMotorsToPosition(double power) {
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPowers(power, power, power, power);
    }

    private void resetEncoders() {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean motorsBusy() {
        return leftFront.isBusy() || leftRear.isBusy() || rightFront.isBusy() || rightRear.isBusy();
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftRear.setPower(bl);
        rightRear.setPower(br);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }
}