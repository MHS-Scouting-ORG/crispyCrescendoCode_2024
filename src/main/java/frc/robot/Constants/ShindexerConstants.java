package frc.robot.Constants;

public final class ShindexerConstants {
    public static final int SHOOTER_PORT_A = 13;
    public static final int SHOOTER_PORT_B = 14;
    public static final int INDEXER_PORT = 10;
    public static final int OPTICAL_SWITCH_PORT = 2;

    public static final double SHOOTER_SPEED = .5;
    public static final double INDEXER_SPEED = .2;
    public static final double MAX_SPEED = .8;
    public static final double RPM_SPEED = SHOOTER_SPEED * 5400;

    public static final double SHOOTER_kP = 0.1;
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0;
}