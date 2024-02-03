// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

  // declared motors
  //private final TalonFX tMotor1;
  //private final TalonFX tMotor2;
  private final CANSparkMax motor1;
  private final CANSparkMax motor2;
  private RelativeEncoder encoder1;
 

  public ShooterSubsystem() {
    // Instantiate motors
    // tMotor1 = new TalonFX(1);
    // tMotor2 = new TalonFX(2);
    motor1 = new CANSparkMax(1, MotorType.kBrushless);
    motor2 = new CANSparkMax(2, MotorType.kBrushless);
    encoder1 = motor1.getEncoder();
  }

  public double getEncoder(){
    return encoder1.getPosition();
  }

  // public double getRPM(){
  //   return tMotor1.getVelocity().getValueAsDouble()*600/2048;
  // }

  public void shooter(double speed) {
    // tMotor1.set(speed);
    // tMotor2.set(speed);
    motor1.set(speed);
    motor2.set(speed);
  }

  public void stop() {
    // tMotor1.stopMotor();
    // tMotor2.stopMotor();
    motor1.stopMotor();
    motor2.stopMotor();
  }
  public void resetEncoder(){
    encoder1.setPosition(0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("ShooterSpeed", getRPM());
    SmartDashboard.putNumber("Shooter Encoder Value", getEncoder());
  }
}