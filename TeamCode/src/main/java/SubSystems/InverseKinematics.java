package SubSystems;

public class InverseKinematics {
    private final double shoulderLength;
    private final double forearmLength;

    public InverseKinematics(double shoulderLength, double forearmLength) {
        this.shoulderLength = shoulderLength;
        this.forearmLength = forearmLength;
    }

    public static class JointAngles {
        public final double shoulderAngle;
        public final double forearmAngle;

        public JointAngles(double shoulderAngle, double forearmAngle) {
            this.shoulderAngle = shoulderAngle;
            this.forearmAngle = forearmAngle;
        }
    }

    /**
     * Calculates the joint angles (in degrees) to reach a given target (x, y) position.
     * Assumes the base of the shoulder is at (0, 0).
     */
    public JointAngles calculateJointAngles(double targetX, double targetY) {
        double distance = Math.sqrt(targetX * targetX + targetY * targetY);
        double maxReach = shoulderLength + forearmLength;

        if (distance > maxReach) {
            throw new IllegalArgumentException("Target is out of reach: (" + targetX + ", " + targetY + ")");
        }

        double angleA = Math.acos(
                (shoulderLength * shoulderLength + distance * distance - forearmLength * forearmLength) /
                        (2 * shoulderLength * distance)
        );

        double angleB = Math.atan2(targetY, targetX);

        double shoulderAngle = Math.toDegrees(angleB - angleA);

        double angleC = Math.acos(
                (shoulderLength * shoulderLength + forearmLength * forearmLength - distance * distance) /
                        (2 * shoulderLength * forearmLength)
        );

        double forearmAngle = 180 - Math.toDegrees(angleC);

        return new JointAngles(shoulderAngle, forearmAngle);
    }

    public static double degreesToTicks(double degrees, double ticksPerRevolution) {
        return (degrees / 360.0) * ticksPerRevolution;
    }
}
        