package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class S_DriveToPositionCommand extends Command {
  SwerveSubsystem swerveSub;
  PIDController xPID, yPID, zPID;
  double desiredX, desiredY, desiredZ, translationTollerance;
  boolean rotation;


  public S_DriveToPositionCommand(SwerveSubsystem swerveSub, double desiredX, double desiredY, double desiredZ, double translationTollerance, boolean rotation) {
    this.swerveSub = swerveSub;
    this.desiredX = (desiredX);
    this.desiredY = (desiredY);
    this.desiredZ = desiredZ;
    this.translationTollerance = translationTollerance;
    this.rotation = rotation;
    xPID = new PIDController(SwerveConstants.AutoConstants.KP_AUTO_TRANSLATION, SwerveConstants.AutoConstants.KI_AUTO_TRANSLATION, SwerveConstants.AutoConstants.KD_AUTO_TRANSLATION);
    yPID = new PIDController(SwerveConstants.AutoConstants.KP_AUTO_TRANSLATION, SwerveConstants.AutoConstants.KI_AUTO_TRANSLATION, SwerveConstants.AutoConstants.KD_AUTO_TRANSLATION);
    zPID = new PIDController(SwerveConstants.AutoConstants.KP_AUTO_ROTATION, SwerveConstants.AutoConstants.KI_AUTO_ROTATION, SwerveConstants.AutoConstants.KD_AUTO_ROTATION);
    zPID.enableContinuousInput(-180, 180);
    xPID.setTolerance(translationTollerance);
    yPID.setTolerance(translationTollerance);
    zPID.setTolerance(SwerveConstants.AutoConstants.ROTATION_TOLLERANCE);

    addRequirements(swerveSub);
  }

  public S_DriveToPositionCommand(SwerveSubsystem swerveSub, double desiredX, double desiredY, double desiredZ, boolean rotation){
    this(swerveSub, desiredX, desiredY, desiredZ, SwerveConstants.AutoConstants.TRANSLATION_TOLLERANCE, rotation);
  }
     
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    SmartDashboard.putString("CURRENT CMD", getName());

    double xSped = xPID.calculate(swerveSub.getPose().getX(), desiredX);
    double ySpeed = yPID.calculate(swerveSub.getPose().getY(), desiredY);
    double zSpeed = !rotation ? 0 : -zPID.calculate(swerveSub.getPose().getRotation().getDegrees(), desiredZ);
    //double zSpeed = !rotation ? 0 : -zPID.calculate((((swerveSub.getRotation2d().getDegrees() % 360) + 360) % 360), desiredZ);
    // zSpeed = 0.01/zSpeed; 

    SmartDashboard.putNumber("xSpeed", xSped);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("zSpeed", zSpeed);


    swerveSub.drive(xSped, ySpeed, zSpeed, true, false);
    
    SmartDashboard.putNumber("xSetpt", Units.metersToFeet(desiredX));
    SmartDashboard.putNumber("ySetpt", Units.metersToFeet(desiredY));
    SmartDashboard.putNumber("zSetpt", desiredZ);

    SmartDashboard.putBoolean("x at setpt", xPID.atSetpoint());
    SmartDashboard.putBoolean("y at setpt", yPID.atSetpoint());
    SmartDashboard.putBoolean("z at setpt", zPID.atSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("CURRENT CMD", "NONE");
    swerveSub.stopModules();
  }

  @Override
  public boolean isFinished() {
    return xPID.atSetpoint() && yPID.atSetpoint() && (zPID.atSetpoint() || !rotation);
  }
}
