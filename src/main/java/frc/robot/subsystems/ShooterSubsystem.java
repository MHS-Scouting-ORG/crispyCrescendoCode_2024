// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShindexerConstants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {

  // declared motors
  private final TalonFX tMotor1;
  private final TalonFX tMotor2;
  private final PIDController shooterPID;
  // private final SimpleMotorFeedforward feedForward;
  private boolean shooterPIDStatus = false;
  private double shooterSpeed = 0; 
  

  public ShooterSubsystem() {
    // Instantiate motors
    tMotor1 = new TalonFX(ShindexerConstants.SHOOTER_PORT_A);
    tMotor2 = new TalonFX(ShindexerConstants.SHOOTER_PORT_B);
    tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(50));
    tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(50));
    tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
    tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
    tMotor1.setInverted(true);
    tMotor2.setInverted(true);
    // feedForward = new SimpleMotorFeedforward(0, getRPM(), tMotor1.getAcceleration().getValueAsDouble());
    shooterPID = new PIDController(ShindexerConstants.SHOOTER_kP, ShindexerConstants.SHOOTER_kI, ShindexerConstants.SHOOTER_kD);
  }

  public double getRPM(){
    return tMotor1.getVelocity().getValueAsDouble()*60;
  }

  // units of distance = meters
  public void shootToSpeaker(double distance){
    if (0.1*distance + .5> 1){
      tMotor1.set(1);
      tMotor2.set(1);
    }else {
      tMotor1.set(0.1*(distance) + .5);
      tMotor2.set(0.1*(distance) + .5);
    }
  }

  public void shooter(double speed) {
    tMotor1.set(speed);
    tMotor2.set(speed);
  }

  public void stop() {
    tMotor1.stopMotor();
    tMotor2.stopMotor();
  }

  public void setSpeed(double speed){
    shooterSpeed = speed;
  }

  public void setPIDStatus(boolean status){
    shooterPIDStatus = status;
  }

  @Override
  public void periodic() {
    if(shooterPIDStatus){
      double velocityError = shooterPID.calculate(getRPM(), ShindexerConstants.RPM_SPEED);
      if(getRPM() < ShindexerConstants.RPM_SPEED){
        shooter(shooterSpeed);
      }
      if(velocityError > ShindexerConstants.RPM_SPEED){
        shooter(shooterSpeed);
      }
    }

    SmartDashboard.putNumber("Shooter Speed", getRPM());
    SmartDashboard.putNumber("Shooter Percentage", shooterSpeed);
  }
}