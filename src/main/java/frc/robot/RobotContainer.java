package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntegrationConstants.OperatorConstants;
import frc.robot.Constants.LimelightHelpers;
import frc.robot.Constants.ShindexerConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.LimelightTurnAlignCmd;
import frc.robot.commands.CrispyPositionCommands.AlignPivotShoot;
import frc.robot.commands.CrispyPositionCommands.AmpPosition;
import frc.robot.commands.CrispyPositionCommands.DownPosition;
import frc.robot.commands.CrispyPositionCommands.FeedPosition;
import frc.robot.commands.ElevatorCommands.ElevatorRestingPositionCmd;
import frc.robot.commands.ElevatorCommands.ElevatorToSetpointCommand;
import frc.robot.commands.ElevatorCommands.ElevatorToTopCmd;
import frc.robot.commands.IntakeCommands.IntakeAuto;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.IntakeCommands.OuttakeCmd;
import frc.robot.commands.PivotCommands.PivotPidAlignCommand;
import frc.robot.commands.PivotCommands.PivotPidCommand;
import frc.robot.commands.PivotCommands.RunToTopLim;
import frc.robot.commands.ShindexerCommands.IndexToShooterAutoCommand;
import frc.robot.commands.ShindexerCommands.IntakeShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ShindexerCommands.IndexToShooterCommand;
import frc.robot.commands.ShindexerCommands.IndexerCommand;
import frc.robot.commands.ShindexerCommands.ShooterCommand;

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
  private final XboxController xboxOp = new XboxController(1);

  //////////////////////////////
  //         TRIGGERS         //
  //////////////////////////////
  private final Trigger opticalTrigger = new Trigger(intakeSubsystem::getOpticalSensor);
  private final Trigger indexTrigger = new Trigger(indexSubsystem::getOpticalSwitch);

  //////////////////////////////
  //      DRIVER BUTTONS      //
  //////////////////////////////
  private final JoystickButton Dy = new JoystickButton(xbox, XboxController.Button.kY.value); 
  private final JoystickButton Dx = new JoystickButton(xbox, XboxController.Button.kX.value);
  private final JoystickButton Da = new JoystickButton(xbox, XboxController.Button.kA.value);
  private final JoystickButton Db = new JoystickButton(xbox, XboxController.Button.kB.value);

  private final JoystickButton DLeftBumper = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton DRightBumper = new JoystickButton(xbox, XboxController.Button.kRightBumper.value);

  private final JoystickButton b_resetNavx = new JoystickButton(xbox, 7); 

  //////////////////////////////
  //     OPERATOR BUTTONS     //
  //////////////////////////////
  private final JoystickButton Oy = new JoystickButton(xboxOp, XboxController.Button.kY.value); 
  private final JoystickButton Ox = new JoystickButton(xboxOp, XboxController.Button.kX.value);
  private final JoystickButton Oa = new JoystickButton(xboxOp, XboxController.Button.kA.value);
  private final JoystickButton Ob = new JoystickButton(xboxOp, XboxController.Button.kB.value);

  private final JoystickButton OLeftBumper = new JoystickButton(xboxOp, XboxController.Button.kLeftBumper.value);
  private final JoystickButton ORightBumper = new JoystickButton(xboxOp, XboxController.Button.kRightBumper.value);
  private final JoystickButton O7Button = new JoystickButton(xboxOp, 7); 
  private final JoystickButton O8Button = new JoystickButton(xboxOp, 8);

  private final Trigger noteSensed = new Trigger(indexSubsystem::getOpticalSwitch);

  //////////////////////////////
  //        AUTO CHOICES      //
  //////////////////////////////
  public SendableChooser<String> autoAllianceChooser = new SendableChooser<>();
  public SendableChooser<String> autoPositionChooser = new SendableChooser<>();
  public SendableChooser<String> autoNameChooser = new SendableChooser<>();

  // public Command PositionA = new A_PositionA(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem); 
  // public Command PositionB = new A_PositionB(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem); 
  // public Command PositionC = new A_PositionC(swerveSubsystem, intakeSubsystem, indexSubsystem, shooterSubsystem, elevatorSubsystem, pivotSubsystem);
  // public Command JustShoot = new A_JustShoot(swerveSubsystem, shooterSubsystem, indexSubsystem);

  public RobotContainer() {
    //AUTO CHOOSER 
    selectAuto();
    registerAutoCommands();

    //CONFIGURE BINDINGS
    configureBindings();

    //DEFAULT COMMANDS
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

        // pivotSubsystem.setDefaultCommand(new RunToTopLim(pivotSubsystem)); //INDEX POS 
        // elevatorSubsystem.setDefaultCommand(new ElevatorRestingPositionCmd(elevatorSubsystem));

        // pivotSubsystem.setDefaultCommand(new ManualPivotCommand(pivotSubsystem, () -> xboxOp.getRightY() * 0.5));

  }

  private void configureBindings() {
    //TRIGGER 
    noteSensed.onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink("limelight")), 
      new WaitCommand(2), 
      new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight"))
    ));
    // noteSensed.onFalse(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")));

    //////////////////////////////
    //          DRIVER          //
    //////////////////////////////
    /* * * BUMPERS * * */
    b_resetNavx.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));

    //INTAKE 
    // START OF DRIVER CODE TO COMMENT 
    // DLeftBumper.whileTrue(  
    //   new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       new ElevatorRestingPositionCmd(elevatorSubsystem),
    //       new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem)
    //     ), 
    //     new InstantCommand(() -> shooterSubsystem.shooter(0.07)),
    //     new RunToTopLim(pivotSubsystem)
    //   )
    // ); 
    // DLeftBumper.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 
    // //SHOOTING AT SET ANGLE 
    // DRightBumper.whileTrue(
    //   new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       new PivotPidCommand(pivotSubsystem, 33), 
    //       new IndexToShooterCommand(shooterSubsystem, indexSubsystem)
    //     ), 
    //     new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")), 
    //     new RunToTopLim(pivotSubsystem))); 
    // // DRightBumper.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX));
    // DRightBumper.whileFalse(new InstantCommand(shooterSubsystem::stop)); 
    // DRightBumper.whileFalse(new InstantCommand(indexSubsystem::stop)); 
    // // DRightBumper.whileFalse(new RunToTopLim(pivotSubsystem)); 

    // /* * * BUTTONS * * */
    // // Dy.onTrue(new PivotPidCommand(pivotSubsystem, 29));
    // //SHOOT WITH CALCULATION 
    // Dx.onTrue(new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new PivotPidAlignCommand(pivotSubsystem), 
    //     new IndexToShooterCommand(shooterSubsystem, indexSubsystem)
    //   ),
    //   new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")))); 
    // Dx.whileTrue(new LimelightTurnAlignCmd(swerveSubsystem, xbox::getLeftY, xbox::getLeftX));
    // //PASSING 
    // Db.onTrue(new ParallelCommandGroup(
    //   new PivotPidCommand(pivotSubsystem, 50), 
    //   new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem, 0.5, 0.8)
    // ));
    // //OUTTAKE 
    // Da.whileTrue(new OuttakeCmd(intakeSubsystem)); 
    // Da.whileFalse(new InstantCommand(intakeSubsystem::stopIntake)); 
    
    // //////////////////////////////
    // //         OPERATOR         //
    // //////////////////////////////
    // /* * * BUMPERS * * */
    // //TUCK 
    // OLeftBumper.onTrue(new DownPosition(elevatorSubsystem, pivotSubsystem)); 
    // //ELEV UP 
    // ORightBumper.onTrue(new ElevatorToTopCmd(elevatorSubsystem)); 

    // /* * * BUTTONS * * */
    // //AMP 
    // Ob.onTrue(new SequentialCommandGroup(
    //   new AmpPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem, shooterSubsystem), 
    //   new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")),
    //   new DownPosition(elevatorSubsystem, pivotSubsystem)
    // )); 
    // //INTAKE THROUGH SHOOTER 
    // Oa.whileTrue(new SequentialCommandGroup(
    //   new IntakeShooterCommand(shooterSubsystem, indexSubsystem), 
    //   new RunToTopLim(pivotSubsystem))); 
    // Oa.whileFalse(new InstantCommand(shooterSubsystem::stop)); 
    // Oa.whileFalse(new InstantCommand(indexSubsystem::stop)); 
    // //SUB SHOT 
    // Ox.onTrue(new ParallelCommandGroup(
    //   new PivotPidCommand(pivotSubsystem, 55), 
    //   new IndexToShooterCommand(shooterSubsystem, indexSubsystem)
    // )); 
    //END OF DRIVER CODE TO COMMENT  
    
    //////////////////////////////
    //        PIT CHECKS        //
    //////////////////////////////
   
    // Edrich pit checks
    DLeftBumper.whileTrue(new IndexerCommand(indexSubsystem));
    DLeftBumper.whileFalse(new InstantCommand(indexSubsystem :: stop));

    DRightBumper.whileTrue(new ShooterCommand(shooterSubsystem));
    DRightBumper.whileFalse(new InstantCommand(shooterSubsystem :: stop));

    Da.whileTrue(new IntakeCmd(intakeSubsystem));
    Da.whileFalse(new InstantCommand(intakeSubsystem :: stopIntake));

    Dy.whileTrue(new OuttakeCmd(intakeSubsystem));
    Dy.whileFalse(new InstantCommand(intakeSubsystem :: stopIntake));

    // Keani Pit checks
    Oy.onTrue(new ElevatorToSetpointCommand(elevatorSubsystem, 70));//(new ElevatorToTopCmd(elevatorSubsystem));
    Oa.onTrue(new ElevatorRestingPositionCmd(elevatorSubsystem));
    Ox.onTrue(new PivotPidCommand(pivotSubsystem, 30));
    Ob.onTrue(new RunToTopLim(pivotSubsystem));
    
  }

  

  public void selectAuto(){
    // autonomousChooser.setDefaultOption("Nothing", new NothingCmd());
    // autonomousChooser.addOption("Position A", PositionA);
    // autonomousChooser.addOption("Position B", PositionB);
    // autonomousChooser.addOption("Position C", PositionC);
    // autonomousChooser.addOption("Just Shoot", JustShoot);

    //PP TESTING 
    // autoAllianceChooser.addOption("Blue", "B");
    // autoAllianceChooser.addOption("Red", "R");

    // autoPositionChooser.addOption("A (amp)", "A");
    // autoPositionChooser.addOption("B", "B"); 
    // autoPositionChooser.addOption("C (source)", "C");

    // autoNameChooser.addOption("Test Path", "Testing");
    // autoNameChooser.addOption("Shoot (3 note)", " Shoot");
    // autoNameChooser.addOption("Troll (disrupt)", " Troll");

    autoNameChooser.addOption("A (4 note)", "A 4 note");
    autoNameChooser.addOption("B (3 note)", "B ToCenter");
    autoNameChooser.addOption("B (rembrandts 5 note)", "B Rev");
    autoNameChooser.addOption("C (3 note)", "CB Shoot");
    // autoNameChooser.addOption("C Rev", "C Rev");

    // SmartDashboard.putData(autoAllianceChooser);
    // SmartDashboard.putData(autoPositionChooser); 
    SmartDashboard.putData(autoNameChooser);
  }

  private void registerAutoCommands() {
    NamedCommands.registerCommand("IntakeCommand", new IntakeAuto(intakeSubsystem)); 
    NamedCommands.registerCommand("StopIntake", new InstantCommand(intakeSubsystem::stopIntake));
    NamedCommands.registerCommand("Wait", new WaitCommand(5));

    NamedCommands.registerCommand("ShootAuto", new ParallelCommandGroup(
      new PivotPidCommand(pivotSubsystem, 58), 
      new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem, 0.4, 0.9)
    ));
    NamedCommands.registerCommand("AlignShootAuto", new ParallelRaceGroup(
      new AlignPivotShoot(pivotSubsystem, shooterSubsystem, indexSubsystem), 
      new LimelightTurnAlignCmd(swerveSubsystem, () -> 0.0, () -> 0.0)
    ));
    NamedCommands.registerCommand("AlignShootParallel", new ParallelDeadlineGroup( 
      new SequentialCommandGroup(
        new WaitCommand(0.25),
        new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem)
      ),
      new PivotPidAlignCommand(pivotSubsystem),
      new LimelightTurnAlignCmd(swerveSubsystem, () -> 0, () -> 0)
    ));
    NamedCommands.registerCommand("AlignShootOneSec", new ParallelDeadlineGroup( 
      new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem, ShindexerConstants.TELEOP_SHOOTER_SPEED, 1), 
      new PivotPidAlignCommand(pivotSubsystem),
      new LimelightTurnAlignCmd(swerveSubsystem, () -> 0, () -> 0) 
    ));
    NamedCommands.registerCommand("FeedPosition", new FeedPosition(elevatorSubsystem, pivotSubsystem, indexSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("RunToTopLim", new RunToTopLim(pivotSubsystem));
    NamedCommands.registerCommand("SetShooter", new InstantCommand(() -> shooterSubsystem.shooter(0.80)));
    // NamedCommands.registerCommand("Index", new IndexAuto(indexSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("Index", new IndexToShooterAutoCommand(shooterSubsystem, indexSubsystem, 0.4, 0.8));
    NamedCommands.registerCommand("SetPivot32", new PivotPidCommand(pivotSubsystem, 33));
    NamedCommands.registerCommand("SetPivot30", new PivotPidCommand(pivotSubsystem, 30));
    NamedCommands.registerCommand("SetPivot41", new PivotPidCommand(pivotSubsystem, 41));
    NamedCommands.registerCommand("SetPivot58", new PivotPidCommand(pivotSubsystem, 58));
  }

  public Command getAutonomousCommand() {
    String autoName = autoNameChooser.getSelected(); 
    return new PathPlannerAuto(autoName); 
    // return new PathPlannerAuto("CB Shoot");


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
