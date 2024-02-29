package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToTopCmd extends Command {

  private ElevatorSubsystem elevSub;

  public ElevatorToTopCmd(ElevatorSubsystem newElevSub) {
    elevSub = newElevSub;
  
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // SmartDashboard.putString("ELEV DONE")
    // FIXME Change encoder setpoint for top
    // TOP IS 56 ENCODER COUNTS 
    elevSub.setSetpoint(56);
  }

  @Override
  public void end(boolean interrupted) {
    elevSub.elevStop();
  }

  @Override
  public boolean isFinished() {
    // SmartDashboard.putString("ELEV DONE", "YESSIR"); 
    return elevSub.getTopLimitSwitch() || elevSub.isAtSetpoint();
  }
}
