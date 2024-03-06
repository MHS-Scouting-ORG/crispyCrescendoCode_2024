// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.commands.ShindexerCommands.IndexToShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class A_PositionB extends SequentialCommandGroup {

  public A_PositionB(SwerveSubsystem swerveSub, UnderIntakeSubsystem intakeSub, IndexerSubsystem indexSub, ShooterSubsystem shooterSub, ElevatorSubsystem elevSub, PivotSubsystem pivotSub) {
    
    addCommands(
      
      new InstantCommand(() -> swerveSub.zeroHeading()),

      new InstantCommand(() -> swerveSub.setZeroOdometer(new Pose2d(0, 0, new Rotation2d(0)))),
      
      //new IndexToShooterCommand(shooterSub, indexSub), // shoots preload into amp
      
      new ParallelCommandGroup(
        
        new S_DriveToPositionCommand(swerveSub, 5, 0, 0, false), // drive to note

        new IntakeCmd(intakeSub)//, // intake note

        //new SequentialCommandGroup(new ElevatorToTransferCmd(elevSub), new PivotPidCommand(pivotSub, 0)) // set to shooting position
      )

      //new IndexToShooterCommand(shooterSub, indexSub) // shoot second preload
    );
  }
}
