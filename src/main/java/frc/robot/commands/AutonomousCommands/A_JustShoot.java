package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ShindexerCommands.IndexToShooterAutoCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class A_JustShoot extends SequentialCommandGroup {

  public A_JustShoot(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IndexerSubsystem indexSubsystem) {
    addCommands(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),

      new InstantCommand(() -> swerveSubsystem.zeroHeading()), 

      // new ElevatorToTransferCmd(elevSub),

      new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem) // shoots preload into amp 
    );
  }
}
