// package frc.robot.commands.AutonomousCommands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.CrispyPositionCommands.FeedToIndexer;
// import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
// import frc.robot.commands.IntakeCommands.IntakeCmd;
// import frc.robot.commands.PivotCommands.PivotPidCommand;
// import frc.robot.commands.ShindexerCommands.IndexToShooterAutoCommand;
// import frc.robot.commands.ShindexerCommands.IndexToShooterCommand;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.UnderIntakeSubsystem;
// import frc.robot.commands.ShindexerCommands.IndexToShooterSlowerCommand;

// public class A_PositionC extends SequentialCommandGroup {

//   public A_PositionC(SwerveSubsystem swerveSub, UnderIntakeSubsystem intakeSub, IndexerSubsystem indexSub, ShooterSubsystem shooterSub, ElevatorSubsystem elevSub, PivotSubsystem pivotSub) {
//     int red = 1;
//     if (DriverStation.getAlliance().get() == Alliance.Red) {
//       red = -1;
//     }
//     addCommands(

//       // new WaitCommand(5), 

//       new InstantCommand(() -> swerveSub.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),

//       new InstantCommand(() -> swerveSub.zeroHeading()), 

//       // new ElevatorToTransferCmd(elevSub),

//       new IndexToShooterSlowerCommand(shooterSub, indexSub), // shoots preload into amp 

//       new InstantCommand(() -> swerveSub.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),

//       new InstantCommand(() -> swerveSub.zeroHeading()), 

//       new S_DriveToPositionCommand(swerveSub, 16, red * 7, 0, false),
      

//       //new S_DriveToPositionCommand(swerveSub, 1, red * 0 , 60, true),

//       new InstantCommand(() -> swerveSub.resetOdometry(new Pose2d(0, 0, new Rotation2d()))),

//       new InstantCommand(() -> swerveSub.zeroHeading())
//     );
//   }
// }
