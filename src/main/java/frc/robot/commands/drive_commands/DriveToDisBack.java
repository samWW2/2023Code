// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDisBack extends CommandBase {
  private  DriveSubsystem drive;
  private double kp = 0.75;
  private Timer timer;
  private PIDController pid = new PIDController(kp, 0, 0);
  boolean alreadyStartedTimer = false;
 // add changes that you have made to blncnew with timer


  
  /** Creates a new DriveToDis. */
  public DriveToDisBack(DriveSubsystem drive, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    this.drive= drive;
    pid.setSetpoint(-setPoint);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   drive.resetEncoders();
   pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidCalculated = pid.calculate(drive.getBothEncoders());
    if(Math.abs(pidCalculated)< 0.2){
      pidCalculated = 0;
    }
    drive.tankDrive(pidCalculated, pidCalculated);
    if(pid.atSetpoint() && !alreadyStartedTimer){
      alreadyStartedTimer = true;
      timer.stop();
      timer.reset();
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }
   
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pid.atSetpoint() && timer.hasElapsed(1) && !drive.isPitchChanged()){
       return true;
    }
    return false;
  }
  
}
