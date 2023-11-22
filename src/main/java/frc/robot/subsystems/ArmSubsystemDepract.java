// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;





public class ArmSubsystemDepract extends SubsystemBase {
  
  private final CANSparkMax armMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final double kDegToMeters = 0.076 * Math.PI ;
  
 private RelativeEncoder armEncoder = armMotor.getEncoder();
 
 


 
  public ArmSubsystemDepract() {
    armEncoder.setPositionConversionFactor(1); //CHANGE THIS
    armMotor.setSmartCurrentLimit(10);
  }


  @Override
  public void periodic() {
    System.out.println(getPosition());


  }
  public void setMotor(double speed) {
    if(speed < 0){
      speed = -speed;
    }
    armMotor.set(-speed);
    
}
public void setMotorNegative(double speed){
  armMotor.set(speed);
}
public CANSparkMax GetArmMotor(){
  return this.armMotor;
}
public double getPosition(){
  return armEncoder.getPosition();
}
public void resetEncoders(){
  armEncoder.setPosition(0);
}
public void setVolt(double volt){
  armMotor.setVoltage(volt);
}
public Boolean isDown(){
  if (getPosition() < 10){
    return true;
  }
  return false;

}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}