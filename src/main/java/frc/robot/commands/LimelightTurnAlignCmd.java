// make a command for each specific april tag and in each command, initialize to a specific pipeline that only detects certain april tags 
package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LimelightTurnAlignCmd extends Command {

  /* * * DECLARATION * * */
  SwerveSubsystem swerveSubs; 
  PIDController anglePID; 

  DoubleSupplier xSupp, ySupp, zSupp; 
  int pipeline;
  Optional<Alliance> ally;

  public LimelightTurnAlignCmd(SwerveSubsystem swerveSubs, DoubleSupplier xSupp, DoubleSupplier ySupp, int pipeline) {
    this.swerveSubs = swerveSubs; 

    this.xSupp = xSupp; 
    this.ySupp = ySupp; 
    this.pipeline = pipeline;
    ally = DriverStation.getAlliance();

    anglePID = new PIDController(0.011, 0, 0);
    anglePID.setTolerance(0.5);

    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", pipeline);
    //swerveSubs.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("pipeline", pipeline);
    SmartDashboard.putString("CURRENT CMD", getName());
    
    SwerveModuleState[] states; 

    // TURN SUPPLIERS INTO DOUBLES 
    double xSpeed = xSupp.getAsDouble(); 
    double ySpeed = ySupp.getAsDouble(); 

    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 

    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 

    
    double rotationSpeed = anglePID.calculate(LimelightHelpers.getTX("limelight"), -4.18);
   
    if(LimelightHelpers.getTV("limelight")){
     if (DriverStation.getAlliance().isPresent() && ((ally.get() == Alliance.Blue && swerveSubs.blueAllianceCheck()) || (ally.get() == Alliance.Red && swerveSubs.redAllianceCheck()))){
        swerveSubs.drive(-MathUtil.applyDeadband(xSpeed, OIConstants.kDriveDeadband), -MathUtil.applyDeadband(ySpeed, OIConstants.kDriveDeadband), rotationSpeed, true, false); 
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("CURRENT CMD", "");
    return false;
  }

   /* * * ADDED METHODS * * */
   public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
  }
  
  private static double modifyAxis(double num) {
    // Square the axis
    num = num * num * Math.signum(num);

    return num;
  }
}