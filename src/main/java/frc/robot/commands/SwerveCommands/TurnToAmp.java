// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAmp extends SequentialCommandGroup {
  /** Creates a new TurnToAmp. */
  public TurnToAmp(SwerveSubsystem swerveSub, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(DriverStation.getAlliance().get() == Alliance.Blue){
        addCommands(new SwerveToHeadCommand(swerveSub, xSpeed, ySpeed, 90));
    } else{
        addCommands(new SwerveToHeadCommand(swerveSub, xSpeed, ySpeed, 270));
    }
    
  }
}
