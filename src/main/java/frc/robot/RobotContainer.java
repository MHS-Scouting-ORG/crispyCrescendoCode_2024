package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntegrationConstants.OperatorConstants;
import frc.robot.Constants.LimelightHelpers;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.LimelightTurnAlignCmd;
import frc.robot.commands.AutonomousCommands.A_JustShoot;
import frc.robot.commands.AutonomousCommands.A_PositionA;
import frc.robot.commands.AutonomousCommands.A_PositionB;
import frc.robot.commands.AutonomousCommands.S_DriveToPositionCommand;
import frc.robot.commands.CrispyPositionCommands.AlignPivotShoot;
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
  private final Trigger indexTrigger = new Trigger(indexSubsystem::getOpticalSwitch);

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
  private final JoystickButton O7Button = new JoystickButton(xboxOp, 7); 
  private final JoystickButton O8Button = new JoystickButton(xboxOp, 8);

  private final Trigger noteSensed = new Trigger(intakeSubsystem::getOpticalSensor);

  //////////////////////////////
  //        AUTO CHOICES      //
  //////////////////////////////
  public SendableChooser<String> autoAllianceChooser = new SendableChooser<>();
  public SendableChooser<String> autoPositionChooser = new SendableChooser<>();
  public SendableChooser<String> autoNameChooser = new SendableChooser<>();

  public Command PositionA = new A_PositionA(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem); 
  public Command PositionB = new A_PositionB(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem); 
  public Command PositionC = new A_PositionC(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem);
  public Command JustShoot = new A_JustShoot(swerveSubsystem, shooterSubsystem, indexSubsystem);

  public RobotContainer() {
    // Configure the trigger bindings
    selectAuto();
    registerAutoCommands();
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

        // pivotSubsystem.setDefaultCommand(new ManualPivotCommand(pivotSubsystem, xboxOp::getLeftY));
        pivotSubsystem.setDefaultCommand(new RunToTopLim(pivotSubsystem)); //INDEX POS 
        elevatorSubsystem.setDefaultCommand(new ElevatorRestingPositionCmd(elevatorSubsystem));

        // pivotSubsystem.setDefaultCommand(new ManualPivotCommand(pivotSubsystem, () -> xboxOp.getRightY() * 0.5));

  }

  private void configureBindings() {
    //DRIVE 
    b_resetNavx.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));

    //DRIVER 
    // DLeftBumper.whileTrue(new IntakeCmd(intakeSubsystem)); 
    // DLeftBumper.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 

    // DRightBumper.whileTrue(new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem));
    // DRightBumper.whileFalse(new InstantCommand(shooterSubsystem::stop)); 

    // DRightBumper.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX, 0));
    // DRightBumper.whileTrue(new ShootingPosition(elevatorSubsystem, pivotSubsystem));
    // DRightBumper.onFalse(new DownPosition(elevatorSubsystem, pivotSubsystem));
     
    // Dx.onTrue(new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem));
    // Da.whileTrue(new OuttakeCmd(intakeSubsystem)); 
    // Da.whileFalse(new InstantCommand(intakeSubsystem::stopIntake));
    // Db.onTrue(new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem)); 
    // Db.onFalse(new DownPosition(elevatorSubsystem, pivotSubsystem));
    // // Dy.whileTrue(new IndexerCommand(indexSubsystem)); 
    // // Dy.whileFalse(new InstantCommand(indexSubsystem::stop)); 
    
    // //OPERATOR 
    // ORightBumper.onTrue(new ElevatorToTopCmd(elevatorSubsystem)); 
    // OLeftBumper.onTrue(new DownPosition(elevatorSubsystem, pivotSubsystem)); 

    // Ob.onTrue(new AmpPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem, shooterSubsystem)); 
    // Oy.onTrue(new RunToTopLim(pivotSubsystem)); 
    // Ox.onTrue(new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem));
    // Oa.whileTrue(new IntakeShooterCommand(shooterSubsystem, indexSubsystem)); 
    // Oa.whileFalse(new InstantCommand(shooterSubsystem::stop)); 

    //TRIGGER 
    noteSensed.whileTrue(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink("limelight")));
    noteSensed.whileFalse(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")));

    indexTrigger.whileTrue(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn("limelight")));
    indexTrigger.whileFalse(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")));

    //TESTING 
    DLeftBumper.whileTrue(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new IntakeCmd(intakeSubsystem), 
          new PivotPidCommand(pivotSubsystem, 30)
        ), 
      new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem))
    ); 
    DLeftBumper.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 

    DRightBumper.whileTrue(new SequentialCommandGroup(
      new PivotPidCommand(pivotSubsystem, 32), 
      new IndexToShooterCommand(shooterSubsystem, indexSubsystem)
    )); 
    DRightBumper.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX, 0));

    DRightBumper.whileFalse(new InstantCommand(shooterSubsystem::stop)); 
    DRightBumper.whileFalse(new InstantCommand(indexSubsystem::stop)); 
    //HAVE TO GO 55 FOR SUBWOOFER SPOT 

    // Dy.onTrue(new ElevatorToTopCmd(elevatorSubsystem)); 
    Dx.onTrue(new AlignPivotShoot(pivotSubsystem, shooterSubsystem, indexSubsystem)); 
    Dx.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX, 0));
    // Db.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX, 0)); 
    Db.onTrue(new SequentialCommandGroup(
      new AmpPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem, shooterSubsystem), 
      new DownPosition(elevatorSubsystem, pivotSubsystem)
    )); 
    Da.whileTrue(new OuttakeCmd(intakeSubsystem)); 
    Da.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 
    

    OLeftBumper.whileTrue(new FeedToIndexer(indexSubsystem, intakeSubsystem)); 

    Ox.onTrue(new DownPosition(elevatorSubsystem, pivotSubsystem));
    // Oy.onTrue(new PivotPidCommand(pivotSubsystem, 30));

    noteSensed.onTrue(new SequentialCommandGroup(
      new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem)
      // new RunToTopLim(pivotSubsystem)
    )); 



  }

  public void selectAuto(){
    // autonomousChooser.setDefaultOption("Nothing", new NothingCmd());
    // autonomousChooser.addOption("Position A", PositionA);
    // autonomousChooser.addOption("Position B", PositionB);
    // autonomousChooser.addOption("Position C", PositionC);
    // autonomousChooser.addOption("Just Shoot", JustShoot);

    //PP TESTING 
    autoAllianceChooser.addOption("Blue", "B");
    autoAllianceChooser.addOption("Red", "R");

    autoPositionChooser.addOption("A (amp)", "A");
    autoPositionChooser.addOption("B", "B"); 
    autoPositionChooser.addOption("C (source)", "C");

    autoNameChooser.addOption("Test Path", "Testing");
    autoNameChooser.addOption("Shoot (3 note)", " Shoot");
    autoNameChooser.addOption("Troll (disrupt)", " Troll");

    SmartDashboard.putData(autoAllianceChooser);
    SmartDashboard.putData(autoPositionChooser); 
    SmartDashboard.putData(autoNameChooser);
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand("IntakeCommand", new IntakeCmd(intakeSubsystem)); 
    NamedCommands.registerCommand("StopIntake", new InstantCommand(intakeSubsystem::stopIntake));
    NamedCommands.registerCommand("DeliverCommand", new DeliverCmd(intakeSubsystem));
    NamedCommands.registerCommand("FakeShooting", new SequentialCommandGroup(
      new InstantCommand(() -> intakeSubsystem.intake(0.6)), 
      new WaitCommand(1), 
      new InstantCommand(intakeSubsystem::stopIntake)
    ));
    NamedCommands.registerCommand("Wait", new WaitCommand(5));

    NamedCommands.registerCommand("ShootAuto", new ParallelCommandGroup(
      new PivotPidCommand(pivotSubsystem, 55), 
      new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem)
    ));
    NamedCommands.registerCommand("AlignShootAuto", new ParallelRaceGroup(
      new AlignPivotShoot(pivotSubsystem, shooterSubsystem, indexSubsystem), 
      new LimelightTurnAlignCmd(swerveSubsystem, () -> 0.0, () -> 0.0, 0)
    ));
    NamedCommands.registerCommand("FeedPosition", new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("RunToTopLim", new RunToTopLim(pivotSubsystem));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autonomousChooser.getSelected();

    // String autoName = autoNameChooser.getSelected(); 
    return new PathPlannerAuto("CB Shoot");


    // return new S_DriveToPositionCommand(swerveSubsystem, 0, 2, 0, false);

    // return new A_PositionB(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem);
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> swerveSubsystem.zeroHeading()),

    //   new InstantCommand(() -> swerveSubsystem.setZeroOdometer(new Pose2d(0, 0, new Rotation2d(0)))),

    //   new S_DriveToPositionCommand(swerveSubsystem, 0, 2, 0, false)
    // ); 
  }

  public Command setElevInit() {
    return setElevInit;
  }

  public void autoInit() {
    swerveSubsystem.setAngle(-60/*swerveSubsystem.getAutoStartingAngle(getAutonomousCommand().getName())*/);
    swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(0.77, 4.51), Rotation2d.fromDegrees(-60))/*PathPlannerAuto.getStaringPoseFromAutoFile(getAutonomousCommand().getName())*/);
  }
}
