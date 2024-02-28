package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntegrationConstants.OperatorConstants;
import frc.robot.commands.S_DriveCommand;
import frc.robot.commands.IntakeCommands.IntakeCmd;
import frc.robot.commands.IntakeCommands.OuttakeCmd;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnderIntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  //////////////////////////////
  //        SUBSYSTEMS        //
  //////////////////////////////
  private UnderIntakeSubsystem intakeSubsystem = new UnderIntakeSubsystem(); 
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem(); 


  //////////////////////////////
  //        CONTROLLERS       //
  //////////////////////////////
  private final XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);

  //////////////////////////////
  //      DRIVER BUTTONS      //
  //////////////////////////////
  private final JoystickButton b_resetNavx = new JoystickButton(xbox, XboxController.Button.kB.value);
  private final JoystickButton b_intake = new JoystickButton(xbox, XboxController.Button.kRightBumper.value); 
  private final JoystickButton b_outtake = new JoystickButton(xbox, XboxController.Button.kLeftBumper.value); 

  //////////////////////////////
  //     OPERATOR BUTTONS     //
  //////////////////////////////

  //////////////////////////////
  //        AUTO CHOICES      //
  //////////////////////////////
  public SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new S_DriveCommand(swerveSubsystem, () -> xbox.getLeftY(), () -> xbox.getLeftX(), () -> -xbox.getRightX(), true));
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    //DRIVE 
    b_resetNavx.onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));

    //INTAKE 
    b_intake.toggleOnTrue(new IntakeCmd(intakeSubsystem)); 
    b_intake.toggleOnFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));

    //OUTTAKE 
    b_outtake.toggleOnTrue(new OuttakeCmd(intakeSubsystem));
    b_outtake.toggleOnFalse(new InstantCommand(() -> intakeSubsystem.stopIntake()));
  }

  public void selectAuto(){
    autonomousChooser.setDefaultOption("Nothing", new NothingCmd());
    SmartDashboard.putData(autonomousChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autonomousChooser.getSelected();
  }
}
