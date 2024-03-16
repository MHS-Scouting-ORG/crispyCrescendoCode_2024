package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.LimelightTurnAlignCmd;
import frc.robot.commands.AutonomousCommands.A_JustShoot;
import frc.robot.commands.AutonomousCommands.A_PositionA;
import frc.robot.commands.AutonomousCommands.A_PositionB;
import frc.robot.commands.AutonomousCommands.S_DriveToPositionCommand;
import frc.robot.commands.CrispyPositionCommands.AmpPosition;
import frc.robot.commands.CrispyPositionCommands.AutomaticPickup;
import frc.robot.commands.CrispyPositionCommands.DownPosition;
import frc.robot.commands.CrispyPositionCommands.FeedPosition;
import frc.robot.commands.CrispyPositionCommands.FeedToIndexer;
import frc.robot.commands.CrispyPositionCommands.ShootingPosition;
import frc.robot.commands.ElevatorCommands.ElevatorRestingPositionCmd;
import frc.robot.commands.ElevatorCommands.ElevatorToSetpointCommand;
import frc.robot.commands.ElevatorCommands.ElevatorToTopCmd;
import frc.robot.commands.ElevatorCommands.ElevatorToTransferCmd;
import frc.robot.commands.ElevatorCommands.ManualElevatorCmd;
import frc.robot.commands.IntakeCommands.DeliverCmd;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.IntakeCommands.OuttakeCmd;
import frc.robot.commands.PivotCommands.ManualPivotCommand;
import frc.robot.commands.PivotCommands.PivotPidAlignCommand;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.commands.PivotCommands.RunToTopLim;
import frc.robot.commands.ShindexerCommands.IndexToShooterAutoCommand;
import frc.robot.commands.ShindexerCommands.IndexToShooterCommand;
import frc.robot.commands.ShindexerCommands.IndexerCommand;
import frc.robot.commands.ShindexerCommands.IntakeShooterCommand;
import frc.robot.commands.ShindexerCommands.ShootAmpCommand;
import frc.robot.commands.ShindexerCommands.ShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutonomousCommands.A_PositionC;
import frc.robot.commands.AutonomousCommands.NothingCmd;

public class RobotContainer {
  //////////////////////////////
  //        SUBSYSTEMS        //
  //////////////////////////////
  private UnderIntakeSubsystem intakeSubsystem = new UnderIntakeSubsystem(); 
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 
  private IndexerSubsystem indexSubsystem = new IndexerSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final Command setElevInit = new InstantCommand(() -> elevatorSubsystem.setSetpoint(elevatorSubsystem.getEnc()));


  //////////////////////////////
  //        CONTROLLERS       //
  //////////////////////////////
  private final XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick joystick = new Joystick(3);
  private final XboxController xboxOp = new XboxController(1);

  //////////////////////////////
  //         TRIGGERS         //
  //////////////////////////////
  private final Trigger opticalTrigger = new Trigger(intakeSubsystem::getOpticalSensor);

  //////////////////////////////
  //      DRIVER BUTTONS      //
  //////////////////////////////
  private final JoystickButton b_resetNavx = new JoystickButton(xbox, 7);
  private final JoystickButton b_rotateAmp = new JoystickButton(xbox, XboxController.Button.kX.value); 
  
  private final JoystickButton b_intake = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value); 
  private final JoystickButton b_outtake = new JoystickButton(xbox, XboxController.Button.kRightBumper.value); 

  private final JoystickButton b_indexerFeed = new JoystickButton(xbox, XboxController.Button.kY.value);

  //////////////////////////////
  //     OPERATOR BUTTONS     //
  //////////////////////////////
  private final JoystickButton b_elevToTop = new JoystickButton(joystick, 3);
  private final JoystickButton b_elevToBottom = new JoystickButton(joystick, 4);

  // private final JoystickButton b_ampShooting = new JoystickButton(xbox, XboxController.Button.kB.value);
  private final JoystickButton b_shootah = new JoystickButton(xbox, XboxController.Button.kB.value);

  //////////////////////////////
  //     TESTING BUTTONS      //
  //////////////////////////////
  private final JoystickButton Dy = new JoystickButton(xbox, XboxController.Button.kY.value); 
  private final JoystickButton Dx = new JoystickButton(xbox, XboxController.Button.kX.value);
  private final JoystickButton Da = new JoystickButton(xbox, XboxController.Button.kA.value);
  private final JoystickButton Db = new JoystickButton(xbox, XboxController.Button.kB.value);

  private final JoystickButton DLeftBumper = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton DRightBumper = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);

  private final JoystickButton Oy = new JoystickButton(xboxOp, XboxController.Button.kY.value); 
  private final JoystickButton Ox = new JoystickButton(xboxOp, XboxController.Button.kX.value);
  private final JoystickButton Oa = new JoystickButton(xboxOp, XboxController.Button.kA.value);
  private final JoystickButton Ob = new JoystickButton(xboxOp, XboxController.Button.kB.value);

  private final JoystickButton OLeftBumper = new JoystickButton(xboxOp, XboxController.Button.kLeftBumper.value);
  private final JoystickButton ORightBumper = new JoystickButton(xboxOp, XboxController.Button.kRightBumper.value);
  private final JoystickButton O_IntakeShooter = new JoystickButton(xboxOp, 8); 

  //////////////////////////////
  //        AUTO CHOICES      //
  //////////////////////////////
  public SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  public Command PositionA = new A_PositionA(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem); 
  public Command PositionB = new A_PositionB(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem); 
  public Command PositionC = new A_PositionC(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem);
  public Command JustShoot = new A_JustShoot(swerveSubsystem, shooterSubsystem, indexSubsystem);

  public RobotContainer() {
    // Configure the trigger bindings
    selectAuto();
    configureBindings();

        swerveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> swerveSubsystem.drive(
                -MathUtil.applyDeadband(xbox.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(xbox.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            swerveSubsystem));

        // pivotSubsystem.setDefaultCommand(new ManualPivotCommand(pivotSubsystem, () -> xboxOp.getRightY() * 0.5));

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
    // b_intake.whileTrue(new IntakeCmd(intakeSubsystem)); 
    // b_intake.whileFalse(new InstantCommand(intakeSubsystem::stopIntake));

    //OUTTAKE 
    // b_outtake.whileTrue(new OuttakeCmd(intakeSubsystem));
    // b_outtake.whileFalse(new InstantCommand(intakeSubsystem::stopIntake));

    //SHINDEXER 
    // b_indexerFeed.onTrue(new ParallelRaceGroup(new DeliverCmd(intakeSubsystem), new IndexerCommand(indexSubsystem)));
    // b_shootah.whileTrue(new IndexToShooterCommand(shooterSubsystem, indexSubsystem)); 

    //ELEVATOR 
    // b_elevToTop.onTrue(new ElevatorToTopCmd(elevatorSubsystem)); 
    // b_elevToBottom.onTrue(new ElevatorRestingPositionCmd(elevatorSubsystem)); 

    //TEST DRIVING 
    /* 
    DLeftBumper.whileTrue(new IntakeCmd(intakeSubsystem)); 
    DLeftBumper.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 

    Da.whileTrue(new OuttakeCmd(intakeSubsystem)); 
    Da.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 

    Dx.onTrue(new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem)); 

    DRightBumper.onTrue(new SequentialCommandGroup(
      new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem), 

      new DownPosition(elevatorSubsystem, pivotSubsystem)
    )); */

    //DRIVER 
    DLeftBumper.whileTrue(new IntakeCmd(intakeSubsystem)); 
    DLeftBumper.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 

    DRightBumper.whileTrue(new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem));
    DRightBumper.whileFalse(new InstantCommand(shooterSubsystem::stop)); 

    DRightBumper.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX, 0));
    DRightBumper.whileTrue(new ShootingPosition(elevatorSubsystem, pivotSubsystem));
    DRightBumper.onFalse(new DownPosition(elevatorSubsystem, pivotSubsystem));
     
    Dx.onTrue(new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem));
    Da.whileTrue(new OuttakeCmd(intakeSubsystem)); 
    Da.whileFalse(new InstantCommand(intakeSubsystem::stopIntake));
    Db.onTrue(new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem)); 
    Db.onFalse(new DownPosition(elevatorSubsystem, pivotSubsystem));
    // Dy.whileTrue(new IndexerCommand(indexSubsystem)); 
    // Dy.whileFalse(new InstantCommand(indexSubsystem::stop)); 
    
    //OPERATOR 
    ORightBumper.onTrue(new ElevatorToTopCmd(elevatorSubsystem)); 
    OLeftBumper.onTrue(new DownPosition(elevatorSubsystem, pivotSubsystem)); 

    Ob.onTrue(new AmpPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem, shooterSubsystem)); 
    Oy.onTrue(new RunToTopLim(pivotSubsystem)); 
    Ox.onTrue(new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem));
    Oa.whileTrue(new IntakeShooterCommand(shooterSubsystem, indexSubsystem)); 
    Oa.whileFalse(new InstantCommand(shooterSubsystem::stop)); 

  }

  public void selectAuto(){
    autonomousChooser.setDefaultOption("Nothing", new NothingCmd());
    autonomousChooser.addOption("Position A", PositionA);
    autonomousChooser.addOption("Position B", PositionB);
    autonomousChooser.addOption("Position C", PositionC);
    autonomousChooser.addOption("Just Shoot", JustShoot);

    SmartDashboard.putData(autonomousChooser);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonomousChooser.getSelected();

    // return new A_PositionB(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem);
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> swerveSubsystem.zeroHeading()),

    //   new InstantCommand(() -> swerveSubsystem.setZeroOdometer(new Pose2d(0, 0, new Rotation2d(0)))),

    //   new S_DriveToPositionCommand(swerveSubsystem, 2, -2, 0, false)
    // ); 
  }

  public Command setElevInit() {
    return setElevInit;
  }
}
