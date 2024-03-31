package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightHelpers;
import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.revrobotics.SparkPIDController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;

public class PivotSubsystem extends SubsystemBase {
  private CANSparkMax pivotMotor;
  private AbsoluteEncoder encoder;
  private DigitalInput limitSwitch;
  private DigitalInput bottomLimitSwitch;
  private DigitalInput bottomLimitSwitch2;

  private PIDController pid;
  private double setpoint;
  private boolean pidOn;
  private double setpointTolerance;

  private double manualSpeed;
  private double maxPidSpeed;

  //private double finalAngle;

  public double horizontalDist;

  private double statsCalcAngle;

  public PivotSubsystem(){
    pivotMotor = new CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    pivotMotor.setInverted(false);
    limitSwitch = new DigitalInput(PivotConstants.PIVOT_TOP_LIMIT);
    bottomLimitSwitch = new DigitalInput(PivotConstants.PIVOT_BOTTOM_LIMIT);
    encoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    bottomLimitSwitch2 = new DigitalInput(PivotConstants.PIVOT_BOTTOM_LIMIT2);
    
    encoder.setZeroOffset(332.5);
    encoder.setPositionConversionFactor(360);
    
    pid = new PIDController(PivotConstants.PIVOT_KP, PivotConstants.PIVOT_KI, PivotConstants.PIVOT_KD);
    setpoint = 0;
    setpointTolerance = 0.5;

    manualSpeed = 0;
    maxPidSpeed = PivotConstants.MAX_SPEED;
    statsCalcAngle = 0;
    horizontalDist = 0;

    pid.enableContinuousInput(0, 360); 
    pid.setTolerance(0.5);
  }

 // sets pivotMotor to brake mode
  public void init(){
    pivotMotor.setIdleMode(IdleMode.kBrake);
  }

  //////////////////////
  // ENCODER METHODS  //
  //////////////////////

  // returns current encoder
  public double returnEncoder(){
    return 360 - encoder.getPosition();
  }

  ///////////////////
  //  PID METHODS //
  //////////////////

  // enable pid for pid commands
  public void enablePid(){
    pidOn = true;
    changeSetpoint(returnEncoder()); 
  }
  // disables pid for pid commands
  public void disablePid(){
    pidOn = false;
  }

    // changes setPoint for the pid 
  public void changeSetpoint(double newSetpoint){
    setpoint = newSetpoint;
  }

  /////////////////////
  //  CHECK METHODS  //
  ////////////////////

  // checks if topLimitSwitch is pressed
  public boolean topLimitSwitchPressed(){
    return !limitSwitch.get();
  }

  // checks if bottomLimitSwitch is pressed
  public boolean bottomLimitSwitchPressed(){
    return !bottomLimitSwitch.get();
  }

  public boolean bottomLimitSwitch2Pressed(){
    return !bottomLimitSwitch2.get();
  }

   // checks if pivot reached the setPoint 
  public boolean atSetpoint(){ 
    double error1 = setpoint - returnEncoder();
    return Math.abs(error1) < setpointTolerance;
  }

  /////////////////////////
  //  MOVEMENT METHODS   //
  ////////////////////////

  // stops the pivotMotor
  public void stopMotor(){
    pivotMotor.set(0);
  }

  // deadzone for joyStick
  public double deadzone(double speed){
    if(Math.abs(speed) < 0.1){
      return 0;
    }
    else{
      return speed;
    }
  }

  //setting the speed manually to the joyStick
  public void setManualSpeed(double speed){
    manualSpeed = deadzone(speed);
  }

  /////////////////////////
  //  LIMELIGHT METHODS  //
  ////////////////////////

  // public double angleSubwooferShot(){
  //   return finalAngle;
  // }

  public double returnCalcAngle (){
    statsCalcAngle = 71 + (-16 * horizontalDist) + (1.58 * Math.pow(horizontalDist, 2));
    // statsCalcAngle /= 2;
    return statsCalcAngle;
  }

  public double returnHorizontalDist(){
    horizontalDist = Units.inchesToMeters(43) / (Math.tan(Units.degreesToRadians(LimelightHelpers.getTY("limelight") + 15)));
    return horizontalDist;
  }
 
  @Override
  public void periodic() {
    //finalAngle = Units.radiansToDegrees(Math.atan((Units.inchesToMeters(43) + Units.inchesToMeters(21))/horizontalDist));

    double pidSpeed = 0;

    if(pidOn){
      pidSpeed = -pid.calculate(returnEncoder(), setpoint);
    }
    else{
      pidSpeed = manualSpeed;
    }

    if(topLimitSwitchPressed() && pidSpeed < 0){
      pidSpeed = 0;
    }
    else if(bottomLimitSwitchPressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(bottomLimitSwitch2Pressed() && pidSpeed > 0){
      pidSpeed = 0;
    }
    else if(pidSpeed > maxPidSpeed){
      pidSpeed = maxPidSpeed;
    }
    else if(pidSpeed < -maxPidSpeed){
      pidSpeed = -maxPidSpeed;
    }

    pivotMotor.set(pidSpeed);

    SmartDashboard.putBoolean("[P] Pid On?", pidOn);
    SmartDashboard.putNumber("[P] Speed", pidSpeed);
    SmartDashboard.putBoolean("[P] Top limit switch pressed?", topLimitSwitchPressed());
    SmartDashboard.putNumber("[P] Degree of shooter", returnEncoder());
    SmartDashboard.putBoolean("[P] Bottom limit switch pressed?", bottomLimitSwitchPressed());
    SmartDashboard.putNumber("[P] distance from limelight", returnHorizontalDist());
    SmartDashboard.putBoolean("[P] at setpoint?", atSetpoint());
    SmartDashboard.putNumber("[P] pid setpoint", setpoint);
    SmartDashboard.putBoolean("[P] Bottom limit switch 2 pressed?", bottomLimitSwitch2Pressed());

    SmartDashboard.putNumber("[P] stats calc angle", returnCalcAngle());
    SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));
  }
}