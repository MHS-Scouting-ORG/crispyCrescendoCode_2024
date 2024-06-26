// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToTransferCmd extends Command {

  public ElevatorSubsystem elevSub;

  //Runs the elevator to the position where we transfer a note from the intake into the indexer

  public ElevatorToTransferCmd(ElevatorSubsystem newElevSub) {
    elevSub = newElevSub;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("TransferPosition?", true);
  }

  @Override
  public void execute() {
    // Transfer/Shooting is 80
    elevSub.setSetpoint(50);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("TransferPosition?", false);
  }

  @Override
  public boolean isFinished() {
    return elevSub.isAtSetpoint();
  }
}
