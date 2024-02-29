// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShindexerConstants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ShindexerConstants;

public class ShooterSubsystem extends SubsystemBase {

  // declared motors
  private final TalonFX tMotor1;
  private final TalonFX tMotor2;
  private final SimpleMotorFeedforward feedForward;
  

  public ShooterSubsystem() {
    // Instantiate motors
    tMotor1 = new TalonFX(ShindexerConstants.SHOOTER_PORT_A);
    tMotor2 = new TalonFX(ShindexerConstants.SHOOTER_PORT_B);
    tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(160));
    tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(160));
    tMotor1.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
    tMotor2.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimitEnable(true));
    tMotor1.setInverted(true);
    tMotor2.setInverted(true);
    feedForward = new SimpleMotorFeedforward(0, getRPS(), tMotor1.getAcceleration().getValueAsDouble());
  }

  public double getRPS(){
    return tMotor1.getVelocity().getValueAsDouble();
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

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Shooter Speed", getRPS());
    SmartDashboard.putNumber("Shooter Percentage", ShindexerConstants.SHOOTER_SPEED);
  }
}