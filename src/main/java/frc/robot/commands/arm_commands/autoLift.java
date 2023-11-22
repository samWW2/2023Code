// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystemDepract;
import com.revrobotics.RelativeEncoder;


public class autoLift extends CommandBase {
  private ArmSubsystemDepract arm;
  private double angle = 90;
  private double kp = 0.1;
  private double kd = 0.035;
  private double ki = 0.2;
  private PIDController pid;
  private Timer timer;
  /** Creates a new autoLift. */
  public autoLift(ArmSubsystemDepract arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    pid = new PIDController(kp, ki, kd);
    pid.setIntegratorRange(0, 0.2);
    this.arm = arm;
    pid.setSetpoint(angle);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    arm.resetEncoders();
    timer.reset();
    
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double pidCalculated = pid.calculate(Math.abs(arm.getPosition()));
   
   if(pidCalculated < 0.3){
    pidCalculated = 0;
  }
    arm.setMotor(pidCalculated);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pid.atSetpoint()){
      return true;
    }
    return false;
  }
}
