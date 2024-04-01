package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.UnderIntakeSubsystem;

public class IntakeAuto extends Command {
  UnderIntakeSubsystem intakeSubsystem; 
  Timer timer; 

  public IntakeAuto(UnderIntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem; 
    timer = new Timer();

    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intake(IntakeConstants.INTAKE_MAXSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.getOpticalSensor() || timer.get() > 2;
  }
}
