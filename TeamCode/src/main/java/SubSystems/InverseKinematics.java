package SubSystems;

public class InverseKinematics {
    private final double shoulderLength;
    private final double forearmLength;
    private final double minShoulderAngle;
    private final double maxShoulderAngle;
    private final double minForearmAngle;
    private final double maxForearmAngle;

    public InverseKinematics(double shoulderLength, double forearmLength,
                            double minShoulderAngle, double maxShoulderAngle,
                            double minForearmAngle, double maxForearmAngle) {
        this.shoulderLength = shoulderLength;
        this.forearmLength = forearmLength;
        this.minShoulderAngle = minShoulderAngle;
        this.maxShoulderAngle = maxShoulderAngle;
        this.minForearmAngle = minForearmAngle;
        this.maxForearmAngle = maxForearmAngle;
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
     * Calculates joint angles for vertical plane movement
     * @param planarDistance Horizontal distance from arm base to target (inches)
     * @param height Vertical offset from arm base to target (inches)
     */
    public JointAngles calculateJointAngles(double planarDistance, double height) {
        double r = Math.sqrt(planarDistance * planarDistance + height * height);
        double maxReach = shoulderLength + forearmLength;
        double minReach = Math.abs(shoulderLength - forearmLength);
        
        // Check reachability
        if (r > maxReach || r < minReach) {
            throw new IllegalArgumentException(
                "Target is out of reach: (" + planarDistance + ", " + height + ")\n" +
                "Min reach: " + minReach + "\", Max reach: " + maxReach + "\""
            );
        }

        // Calculate angles using law of cosines
        double angleA = Math.acos(
            (shoulderLength * shoulderLength + r * r - forearmLength * forearmLength) /
            (2 * shoulderLength * r)
        );

        double angleB = Math.atan2(height, planarDistance);

        double shoulderAngle = Math.toDegrees(angleB + angleA);
        double angleC = Math.acos(
            (shoulderLength * shoulderLength + forearmLength * forearmLength - r * r) /
            (2 * shoulderLength * forearmLength)
        );

        double forearmAngle = 180 - Math.toDegrees(angleC);

        // Apply joint limits
        shoulderAngle = Math.max(minShoulderAngle, Math.min(maxShoulderAngle, shoulderAngle));
        forearmAngle = Math.max(minForearmAngle, Math.min(maxForearmAngle, forearmAngle));
        
        return new JointAngles(shoulderAngle, forearmAngle);
    }

    public static double degreesToTicks(double degrees, double ticksPerRevolution, double shoulderGearRatio) {
        return (degrees / 360.0) * ticksPerRevolution;
    }
    
    /**
     * Gravity compensation model for backdrivable arms
     * @param shoulderAngle Current shoulder angle (degrees)
     * @param payloadWeight Weight of payload in pounds
     * @return Additional power factor (0.0-1.0)
     */
    public double gravityCompensationFactor(double shoulderAngle, double payloadWeight) {
        // Center of mass at 60% of shoulder length
        double comDistance = 0.6 * shoulderLength;
        double torque = payloadWeight * comDistance * Math.sin(Math.toRadians(shoulderAngle));
        
        // Convert torque to power factor (0.5 = 50% additional power)
        return Math.min(0.5, torque * 0.3); // Tune the 0.3 factor based on testing
    }
}