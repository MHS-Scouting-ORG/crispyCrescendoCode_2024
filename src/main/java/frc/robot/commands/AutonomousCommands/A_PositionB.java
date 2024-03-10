// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CrispyPositionCommands.FeedToIndexer;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.IntakeCommands.DeliverCmd;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.commands.ShindexerCommands.IndexToShooterAutoCommand;
import frc.robot.commands.ShindexerCommands.IndexToShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class A_PositionB extends SequentialCommandGroup {

  //80 ELEV RIGHT TO MID OF SUBWOOFER 
  public A_PositionB(SwerveSubsystem swerveSub, UnderIntakeSubsystem intakeSub, IndexerSubsystem indexSub, ShooterSubsystem shooterSub, ElevatorSubsystem elevSub, PivotSubsystem pivotSub) {
    int red = 1;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      red = -1;
    }
    addCommands(

        new InstantCommand(() -> swerveSub.zeroHeading()),

        new InstantCommand(() -> swerveSub.setZeroOdometer(new Pose2d(0, 0, new Rotation2d(0)))),

        new IndexToShooterAutoCommand(shooterSub, indexSub), // shoots preload into amp

        new ParallelCommandGroup(

            new S_DriveToPositionCommand(swerveSub, 5, red * 0, 0, false), // drive to note

            new IntakeCmd(intakeSub), // intake note

            // new SequentialCommandGroup(new ElevatorToTransferCmd(elevSub),
            new PivotPidCommand(pivotSub, 33)// ) // set to shooting position
        ),

        new ParallelRaceGroup(
          new DeliverCmd(intakeSub),

          new IndexToShooterAutoCommand(shooterSub, indexSub)
        )
    );
  }
}
