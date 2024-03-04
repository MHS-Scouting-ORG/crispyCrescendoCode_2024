// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShindexerConstants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

public class ShooterSubsystem extends SubsystemBase {

  // declared motors
  private final TalonFX tMotor1;
  private final TalonFX tMotor2;
  private final PIDController shooterPID;
  private boolean shooterPIDStatus = false;

  public ShooterSubsystem() {
    // Instantiate motors
    tMotor1 = new TalonFX(ShindexerConstants.SHOOTER_PORT_A);
    tMotor2 = new TalonFX(ShindexerConstants.SHOOTER_PORT_B);
    tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    // tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyTimeThreshold(1.275));
    // tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyTimeThreshold(1.275));
    tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(false));
    tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(false));
    tMotor1.setInverted(true);
    tMotor2.setInverted(true);
    shooterPID = new PIDController(0.1, 0, 0);
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
    ShindexerConstants.SHOOTER_SPEED = speed;
  }

  public void setPIDStatus(boolean status){
    shooterPIDStatus = status;
  }

  @Override
  public void periodic() {
    if(shooterPIDStatus){
      double velocityError = shooterPID.calculate(getRPM(), ShindexerConstants.RPM_SPEED_LIMIT);
      if(velocityError > ShindexerConstants.RPM_SPEED_LIMIT){
        shooter(ShindexerConstants.MAX_SPEED);
      }
    }

    SmartDashboard.putNumber("Shooter Speed", getRPM());
    SmartDashboard.putNumber("Shooter Current limit", tMotor1.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Voltage", tMotor1.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Percentage", ShindexerConstants.SHOOTER_SPEED);
    SmartDashboard.putNumber("Shooter RPM Limit" , ShindexerConstants.RPM_SPEED_LIMIT);
  }
}