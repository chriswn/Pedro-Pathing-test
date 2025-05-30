package AUTO;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import Constants.ArmConstants;
import SubSystems.InverseKinematics;
import SubSystems.Arm.ArmMovement;

@TeleOp(name = "IK Test - Full System Test", group = "Test")
public class IKTestOpMode extends LinearOpMode {

    // Arm dimensions (match your physical setup)
    private final double SHOULDER_LENGTH = 10.0;  // inches
    private final double FOREARM_LENGTH = 10.0;   // inches
    
    // Joint limits (degrees)
    private final double MIN_SHOULDER_ANGLE = 0;
    private final double MAX_SHOULDER_ANGLE = 135;
    private final double MIN_FOREARM_ANGLE = 0;
    private final double MAX_FOREARM_ANGLE = 160;
    
    // Control parameters
    private final double ANGLE_INCREMENT = 2.0;     // degrees per update
    private final double SPEED_SCALING = 0.5;       // Slower for precision

    @Override
    public void runOpMode() {
        // Initialize Inverse Kinematics with joint limits
        InverseKinematics ik = new InverseKinematics(
            SHOULDER_LENGTH, FOREARM_LENGTH,
            MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE,
            MIN_FOREARM_ANGLE, MAX_FOREARM_ANGLE
        );
        
        ArmMovement arm = new ArmMovement(hardwareMap, telemetry);
        
        telemetry.setAutoClear(false);
        telemetry.addLine("IK TEST - READY");
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick Y: Move Shoulder");
        telemetry.addLine("Right Stick Y: Move Forearm");
        telemetry.addLine("A: Open Gripper");
        telemetry.addLine("B: Close Gripper");
        telemetry.addLine("X: Test IK Position (12in, 0in)");
        telemetry.addLine("Y: Test IK Position (0in, -8in)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Test gripper controls
            if (gamepad1.a) {
                arm.openGripper();
                sleep(200);
            }
            if (gamepad1.b) {
                arm.closeGripper();
                sleep(200);
            }
            
            // Test IK positions
            if (gamepad1.x) {
                testIKPosition(ik, arm, 12.0, 0.0);  // Forward position
            }
            if (gamepad1.y) {
                testIKPosition(ik, arm, 0.0, -8.0);  // Down position
            }
            
            // Manual joint control
            manualJointControl(ik, arm);
            
            telemetry.update();
            sleep(20);
        }
    }
    
    private void manualJointControl(InverseKinematics ik, ArmMovement arm) {
        // Adjust angles based on joystick input
        double shoulderAngleDelta = -gamepad1.left_stick_y * ANGLE_INCREMENT * SPEED_SCALING;
        double forearmAngleDelta = -gamepad1.right_stick_y * ANGLE_INCREMENT * SPEED_SCALING;
        
        // Get current angles (simulated)
        double shoulderAngle = Range.clip(shoulderAngleDelta, MIN_SHOULDER_ANGLE, MAX_SHOULDER_ANGLE);
        double forearmAngle = Range.clip(forearmAngleDelta, MIN_FOREARM_ANGLE, MAX_FOREARM_ANGLE);
        
        // Convert angles to ticks using gear ratios
        int shoulderTicks = (int) InverseKinematics.degreesToTicks(
            shoulderAngle, 
            ArmConstants.SHOULDER_TICKS_PER_REV,
            ArmConstants.SHOULDER_GEAR_RATIO
        );
        
        int forearmTicks = (int) InverseKinematics.degreesToTicks(
            forearmAngle, 
            ArmConstants.FOREARM_TICKS_PER_REV,
            ArmConstants.FOREARM_GEAR_RATIO
        );
        
        // Apply hardware compensation
        shoulderTicks += ArmConstants.SHOULDER_BACKLASH_COMP;
        forearmTicks += ArmConstants.FOREARM_BACKLASH_COMP;
        
        // Move arm
        arm.moveArmToPosition(shoulderTicks, forearmTicks);
        
        // Display information
        telemetry.addLine("MANUAL JOINT CONTROL");
        telemetry.addData("Shoulder Angle", "%.1f°", shoulderAngle);
        telemetry.addData("Forearm Angle", "%.1f°", forearmAngle);
        telemetry.addData("Shoulder Ticks", shoulderTicks);
        telemetry.addData("Forearm Ticks", forearmTicks);
        telemetry.addLine("---------------------");
    }
    
    private void testIKPosition(InverseKinematics ik, ArmMovement arm, double x, double z) {
        try {
            telemetry.addLine("TESTING IK POSITION: (" + x + "in, " + z + "in)");
            telemetry.update();
            
            // Calculate joint angles
            InverseKinematics.JointAngles angles = ik.calculateJointAngles(x, z);
            
            // Convert to ticks
            int shoulderTicks = (int) InverseKinematics.degreesToTicks(
                angles.shoulderAngle, 
                ArmConstants.SHOULDER_TICKS_PER_REV,
                ArmConstants.SHOULDER_GEAR_RATIO
            ) + ArmConstants.SHOULDER_BACKLASH_COMP;
            
            int forearmTicks = (int) InverseKinematics.degreesToTicks(
                angles.forearmAngle, 
                ArmConstants.FOREARM_TICKS_PER_REV,
                ArmConstants.FOREARM_GEAR_RATIO
            ) + ArmConstants.FOREARM_BACKLASH_COMP;
            
            // Move arm
            arm.moveArmToPosition(shoulderTicks, forearmTicks);
            sleep(1000); // Wait for movement
            
            // Display results
            telemetry.addLine("IK RESULTS");
            telemetry.addData("Target Position", "X: %.1fin, Z: %.1fin", x, z);
            telemetry.addData("Shoulder Angle", "%.1f°", angles.shoulderAngle);
            telemetry.addData("Forearm Angle", "%.1f°", angles.forearmAngle);
            telemetry.addData("Shoulder Ticks", shoulderTicks);
            telemetry.addData("Forearm Ticks", forearmTicks);
            telemetry.addLine("---------------------");
            
        } catch (IllegalArgumentException e) {
            telemetry.addLine("IK ERROR: " + e.getMessage());
            telemetry.addLine("---------------------");
        }
    }
}