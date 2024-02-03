package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  
  private final CANSparkMax indexMotor;
  private final DigitalInput opticalSwitch;
  private RelativeEncoder indexEncoder;

  public IndexerSubsystem() {
    indexMotor = new CANSparkMax(3, MotorType.kBrushless);
    opticalSwitch = new DigitalInput(9);
    indexEncoder = indexMotor.getEncoder();
  }

  public double getIndexerEnc(){ return indexEncoder.getPosition();}
  public void resetIndexerEnc(){ indexEncoder.setPosition(0);}

  public boolean getOpticalSwitch(){
    return opticalSwitch.get();
  }

  public void index(){
    indexMotor.set(.5);
  }

  public void stop(){
    indexMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Optical Switch", getOpticalSwitch());
    SmartDashboard.putNumber("Indexer Speed", indexMotor.get());
    SmartDashboard.putNumber("Indexer Encoder Value", getIndexerEnc());
  }
}