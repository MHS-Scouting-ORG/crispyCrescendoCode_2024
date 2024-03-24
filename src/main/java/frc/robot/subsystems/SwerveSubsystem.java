// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveUtils;

public class SwerveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private MAXSwerveModule[] modules = new MAXSwerveModule[] {
    m_frontLeft, 
    m_frontRight, 
    m_rearLeft, 
    m_rearRight
  };
  

  // The gyro sensor
  private final AHRS navx = new AHRS();
  private String m_autoName = ""; 

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //Blue and red alliance tags
  private int blueAllianceTags[] = new int[4];
  private int redAllianceTags[] = new int[4];

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      (Rotation2d.fromDegrees(navx.getAngle()).unaryMinus()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public SwerveSubsystem() {
    
    blueAllianceTags = new int[]{1,2,6,7};
    redAllianceTags = new int[]{4,5,9,10};

    //AUTO BUILDER FROM TORBOTS
    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier

      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)

      this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(2.0, 0.0, 0.003), // Rotation PID constants
        5.5, // Max module speed, in m/s
        (Math.hypot(SwerveConstants.DriveConstants.kWheelBase, SwerveConstants.DriveConstants.kTrackWidth)) / 2, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),


      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red
        // alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },

      this
    );
  }

  
  public boolean blueAllianceCheck(){
    for(int i = 0; i < blueAllianceTags.length; i++){
      if(LimelightHelpers.getFiducialID("limelight") == blueAllianceTags[i]){
        return true;
      }
    }
    return false;
  }

  public boolean redAllianceCheck(){
    for(int i = 0; i < redAllianceTags.length; i++){
      if(LimelightHelpers.getFiducialID("limelight") == redAllianceTags[i]){
        return true;
      }
    }
    return false;
  }

  // public double getAutoStartingAngle(String autoName) {
  //   m_autoName = autoName;
  //    if (!DriverStation.getAlliance().isEmpty()) {
  //     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
  //       SmartDashboard.putNumber("Red angle", PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().unaryMinus().getDegrees());
  //       return 180 + GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName)).getRotation().getDegrees();
  //     } else {
  //       SmartDashboard.putNumber("Blue angle", 180 + PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().unaryMinus().getDegrees());
  //       return PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees();
  
  //     }
  //   } else {
  //     return 0;
  //   }
  // }

  // public double gyroWithOffset() { //TORBOTS TEST CODE 

  //   if (!DriverStation.getAlliance().isEmpty()) {
  //     double offset;
  //     if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
  //       offset = GeometryUtil.flipFieldPose(PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName)).getRotation().getDegrees();
  //       // if(offset == 180){
  //       //   offset = 0;
  //       // }
  //     } else {
  //       offset = PathPlannerAuto.getStaringPoseFromAutoFile(m_autoName).getRotation().getDegrees();
  //     }
  //     return (-navx.getAngle() + 0);
  //   } else {
  //     return 0;
  //   }
    
  // }

  public void setAngle(double angle) {
    navx.setAngleAdjustment(angle);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        (Rotation2d.fromDegrees(navx.getAngle()).unaryMinus()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_frontLeft.print();
    m_rearLeft.print(); 
    m_rearRight.print(); 
    m_frontRight.print();

    SmartDashboard.putNumber("Navx", navx.getAngle());
    SmartDashboard.putString("POSE", getPose().toString());
    // SmartDashboard.putNumber("auto starting ang", getAutoStartingAngle(m_autoName));

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return m_odometry.getPoseMeters().getRotation().unaryMinus();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        (Rotation2d.fromDegrees(navx.getAngle()).unaryMinus()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void setZeroOdometer(Pose2d pose) {
    m_odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getSpeeds() {
    return SwerveConstants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = SwerveConstants.DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, navx.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[modules.length]; 
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void stopModules() {
    m_frontLeft.stopModule();
    m_frontRight.stopModule();
    m_rearLeft.stopModule();
    m_rearRight.stopModule();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navx.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
