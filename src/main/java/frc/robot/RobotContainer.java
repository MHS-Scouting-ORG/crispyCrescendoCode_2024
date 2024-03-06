package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntegrationConstants.OperatorConstants;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.AutonomousCommands.A_PositionB;
import frc.robot.commands.AutonomousCommands.S_DriveToPositionCommand;
import frc.robot.commands.CrispyPositionCommands.AutomaticPickup;
import frc.robot.commands.CrispyPositionCommands.FeedToIndexer;
import frc.robot.commands.ElevatorCommands.ElevatorRestingPositionCmd;
import frc.robot.commands.ElevatorCommands.ElevatorToTopCmd;
import frc.robot.commands.IntakeCommands.DeliverCmd;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.IntakeCommands.OuttakeCmd;
import frc.robot.commands.ShindexerCommands.IndexToShooterCommand;
import frc.robot.commands.ShindexerCommands.IndexerCommand;
import frc.robot.commands.ShindexerCommands.ShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  //////////////////////////////
  //        SUBSYSTEMS        //
  //////////////////////////////
  private UnderIntakeSubsystem intakeSubsystem = new UnderIntakeSubsystem(); 
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 
  private IndexerSubsystem indexSubsystem = new IndexerSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();


  //////////////////////////////
  //        CONTROLLERS       //
  //////////////////////////////
  private final XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick joystick = new Joystick(0);

  //////////////////////////////
  //         TRIGGERS         //
  //////////////////////////////
  private final Trigger opticalTrigger = new Trigger(intakeSubsystem::getOpticalSensor);

  //////////////////////////////
  //      DRIVER BUTTONS      //
  //////////////////////////////
  private final JoystickButton b_resetNavx = new JoystickButton(xbox, XboxController.Button.kA.value);
  private final JoystickButton b_rotateAmp = new JoystickButton(xbox, XboxController.Button.kX.value); 
  
  private final JoystickButton b_intake = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value); 
  private final JoystickButton b_outtake = new JoystickButton(xbox, XboxController.Button.kRightBumper.value); 

  private final JoystickButton b_indexerFeed = new JoystickButton(xbox, XboxController.Button.kX.value);

  //////////////////////////////
  //     OPERATOR BUTTONS     //
  //////////////////////////////
  private final JoystickButton b_elevToTop = new JoystickButton(joystick, 3);
  private final JoystickButton b_elevToBottom = new JoystickButton(joystick, 4);

  private final JoystickButton b_ampShooting = new JoystickButton(xbox, XboxController.Button.kB.value);
  private final JoystickButton b_shootah = new JoystickButton(xbox, XboxController.Button.kY.value);

  //////////////////////////////
  //        AUTO CHOICES      //
  //////////////////////////////
  public SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

        swerveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> swerveSubsystem.drive(
                -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.
                applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            swerveSubsystem));

  }

  private void configureBindings() {
    //DRIVE 
    b_resetNavx.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));

    //OPTICAL TRIGGER AUTOMATIC 
    // opticalTrigger.onTrue(
    //   //new AutomaticPickup(intakeSubsystem, elevatorSubsystem, indexSubsystem)
    //   new SequentialCommandGroup(
    //     new ElevatorToTopCmd(elevatorSubsystem), //goes to mid position to pick up note from indexer 

    //     new FeedToIndexer(indexSubsystem, intakeSubsystem), //feeds note from intake to indexer 

    //     new ElevatorStoragePositionCmd(elevatorSubsystem) 

    // ));

    // b_ampShooting.onTrue(
    //   new SequentialCommandGroup(
    //     new InstantCommand(() -> shooterSubsystem.shooter(ShindexerConstants.SHOOTER_SPEED)),

    //     new ElevatorToTopCmd(elevatorSubsystem), //goes to mid position to pick up note from indexer 

    //     new FeedToIndexer(indexSubsystem, intakeSubsystem), //feeds note from intake to indexer 

    //     //FIXME align command for pivot should be here

    //     new IndexToShooterCommand(shooterSubsystem, indexSubsystem)
    // ));

    //INTAKE 
    b_intake.whileTrue(new IntakeCmd(intakeSubsystem)); 
    b_intake.whileFalse(new InstantCommand(intakeSubsystem::stopIntake));

    //OUTTAKE 
    b_outtake.whileTrue(new OuttakeCmd(intakeSubsystem));
    b_outtake.whileFalse(new InstantCommand(intakeSubsystem::stopIntake));

    //SHINDEXER 
    b_indexerFeed.onTrue(new ParallelRaceGroup(new DeliverCmd(intakeSubsystem), new IndexerCommand(indexSubsystem)));
    b_shootah.whileTrue(new IndexToShooterCommand(shooterSubsystem, indexSubsystem)); 


    //ELEVATOR 
    b_elevToTop.onTrue(new ElevatorToTopCmd(elevatorSubsystem)); 
    b_elevToBottom.onTrue(new ElevatorRestingPositionCmd(elevatorSubsystem)); 
    
  }

  public void selectAuto(){
    autonomousChooser.setDefaultOption("Nothing", new NothingCmd());
    SmartDashboard.putData(autonomousChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autonomousChooser.getSelected();

    return new A_PositionB(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, null);
    /* new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.zeroHeading()),

      new InstantCommand(() -> swerveSubsystem.setZeroOdometer(new Pose2d(0, 0, new Rotation2d(0)))),

      new S_DriveToPositionCommand(swerveSubsystem, 0, 0, 90, true)
    );*/ 
  }
}
