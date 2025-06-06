package Constants;

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

    // Gear ratios
    public static final double SHOULDER_GEAR_RATIO = 5.0;   // 5:1 reduction
    public static final double FOREARM_GEAR_RATIO = 10.0;   // Worm gear reduction
    public static final int SHOULDER_HOME_POSITION =1;
    public static final int FOREARM_HOME_POSITION = 1 ;
}
