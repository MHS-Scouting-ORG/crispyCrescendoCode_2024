package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightHelpers;
import frc.robot.subsystems.PivotSubsystem;

public class PivotPidAlignCommand extends Command {
  PivotSubsystem pivotSubs;

  public PivotPidAlignCommand(PivotSubsystem pivotsub) {
    pivotSubs = pivotsub;
    addRequirements(pivotSubs);
  }

  @Override
  public void initialize() {
    pivotSubs.init();
    pivotSubs.enablePid();
    // pivotSubs.changeSetpoint((int) (pivotSubs.returnCalcAngle()));
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("limelight") || pivotSubs.returnHorizontalDist() > 3.75 && pivotSubs.returnHorizontalDist() < 4.6) {
      pivotSubs.changeSetpoint(33);
    } else {
      pivotSubs.changeSetpoint((int) (pivotSubs.returnCalcAngle()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubs.stopMotor();
  }

  @Override
  public boolean isFinished() {
    // return pivotSubs.returnEncoder() == 360 - pivotSubs.returnCalcAngle();
    return pivotSubs.atSetpoint();
  }
}
