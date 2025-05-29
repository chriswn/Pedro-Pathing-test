package constants;

public class ArmConstants {
    // Core Hex on Forearm
    public static final double FOREARM_TICKS_PER_REV = 288.0;
    public static final double FOREARM_TICKS_PER_DEGREE = FOREARM_TICKS_PER_REV / 360.0;

    // NeveRest or other motor on Shoulder
    public static final double SHOULDER_TICKS_PER_REV = 560.0;
    public static final double SHOULDER_TICKS_PER_DEGREE = SHOULDER_TICKS_PER_REV / 360.0;

    // Compensation for mechanical play
    public static final int FOREARM_BACKLASH_COMP = 150;
    public static final int SHOULDER_BACKLASH_COMP = 50;
}
