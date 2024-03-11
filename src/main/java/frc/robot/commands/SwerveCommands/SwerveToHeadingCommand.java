package frc.robot.commands.SwerveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveToHeadingCommand extends Command {

  /* * * DECLARATION * * */
  SwerveSubsystem swerveSubsystem; 
  double desiredHeading; 
  PIDController rotationPID; 

  DoubleSupplier xSupp, ySupp, zSupp; 

  public SwerveToHeadingCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSupp, DoubleSupplier ySupp, DoubleSupplier zSupp, double desiredHeading) {
    this.swerveSubsystem = swerveSubsystem; 
    this.desiredHeading = desiredHeading; 

    this.xSupp = xSupp; 
    this.ySupp = ySupp; 
    this.zSupp = zSupp; 

    rotationPID = new PIDController(SwerveConstants.DriveConstants.kRotationP, SwerveConstants.DriveConstants.kRotationI, SwerveConstants.DriveConstants.kRotationD);
    rotationPID.setTolerance(SwerveConstants.DriveConstants.kRotationTolerance);
    rotationPID.enableContinuousInput(-180, 180);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //swerveSubsystem.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("CURRENT CMD", getName());

    // TURN SUPPLIERS INTO DOUBLES 
    double xSpeed = xSupp.getAsDouble();
    double ySpeed = ySupp.getAsDouble();
    double zSpeed = zSupp.getAsDouble();

    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed);

    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

        /* * * WAIALUA DESIRED ANGLE MODIFICATIONS * * */
    //UNTESTED BE CAREFUL
    // if (zSpeed != 0) { //if rotation js is being moved 
    //   desiredHeading = swerveSubsystem.getRotation2d().getDegrees(); //used to be .getYaw360();
    // }

    // swerveSubsystem.setdesiredHeading(desiredHeading);
   
    desiredHeading += zSpeed; 
    desiredHeading = (desiredHeading + 360) % 360; //makes the desired angle positive and b/w 0 - 360
    double angleToDesired = -wrap(swerveSubsystem.getRotation2d().getDegrees(), desiredHeading); 
    double rotationSpeed = rotationPID.calculate(desiredHeading, ((swerveSubsystem.getRotation2d().getDegrees() % 360) + 360) % 360);
    // double rotationSpeed = angleToDesired / 90; // makeshift PID that doesnt work 
    // apply range -1 to 1
    if (rotationSpeed > 1) rotationSpeed = 1;
    if (rotationSpeed < -1) rotationSpeed = -1;

    swerveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("CURRENT CMD", "");
    return false;
  }

   /* * * ADDED METHODS * * */
   public double deadzone(double num){
    return Math.abs(num) > OIConstants.kDriveDeadband ? num : 0;
  }
  
  private static double modifyAxis(double num) {
    // Square the axis
    num = num * num * Math.signum(num);

    return num;
  }

  //waialua wrap method from Conversions.java 
  public static double wrap(double angle1, double angle2) {
    double difference = angle1 - angle2;
    if (difference > 180) {
        difference -= 360;
        //difference = -difference;
    } else if (difference < -180) {
        difference += 360;
        //difference = -difference;
    }
    return difference;
  }
}