package frc.robot.Constants;

public final class HangConstants {
    //PORTS 
    public static final int HANG_MOTOR_PORT1 = 3;
    public static final int HANG_MOTOR_PORT2 = 4;
    public static final int TOP_LS_PORT = 9;
    public static final int BOTTOM_LS_PORT = 2;

    //SPEEDS 
    public static final double SPEED_CAP = 0.5;
    public static final double TOP_ENC_LIMIT = 50;
    public static final double BOTTOM_ENC_LIMIT = -50;

    //PID 
    public static final double HANG_KP = 0.01;
    public static final double HANG_KI = 0;
    public static final double HANG_KD = 0;
}