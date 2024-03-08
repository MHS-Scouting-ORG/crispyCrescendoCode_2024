package frc.robot.Constants;

public class ElevatorConstants {
  public static final int ELEVATOR_MOTOR_PORT = 12;
    public static final int TOP_LS_PORT = 3;
    public static final int BOTTOM_LS_PORT = 0;

    //FIXME Up speed after testing
    public static final double SPEED_CAP = 0.6;//0.875;

    public static final double ELEV_KP = 0.03;
    public static final double ELEV_KI = 0.001;
    public static final double ELEV_KD = 0;

    public static final int SMART_CURRENT_LIMIT = 20;
    public static final int PID_TOLERANCE = 1;
  }