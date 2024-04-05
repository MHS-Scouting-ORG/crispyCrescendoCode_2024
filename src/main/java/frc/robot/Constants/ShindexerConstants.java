package frc.robot.Constants;

public final class ShindexerConstants {
    public static final int SHOOTER_PORT_A = 13;
    public static final int SHOOTER_PORT_B = 14;
    public static final int INDEXER_PORT = 10;
    public static final int OPTICAL_SWITCH_PORT = 1;
    public static final double MAX_RPM = 5000;
    public static final double TELEOP_SHOOTER_SPEED = 0.75; 
    public static double SHOOTER_SPEED = .75;
    public static double INDEXER_SPEED = 1;
    public static final double MAX_SPEED = .9;
    public static final double RPM_SPEED_LIMIT = MAX_RPM * SHOOTER_SPEED;
    public static final double RPM_AMP_SPEED_LIMIT = MAX_RPM * 0.095; 
    public static final double RPM_AUTO_SPEED_LIMIT = MAX_RPM * 0.9; 
}