// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToDis extends CommandBase {
  private  DriveSubsystem drive;
  private double kp = 0.7;
  private double ki = 0.3;
  private Timer timer;
  private PIDController pid = new PIDController(kp, ki, 0);
  boolean alreadyStartedTimer = false;


  double setPoint;
 // add changes that you have made to blncnew with timer


  
  /** Creates a new DriveToDis. */
  public DriveToDis(DriveSubsystem drive, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();
    this.drive= drive;
    this.setPoint = setPoint;
    pid.setSetpoint(setPoint);
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
 ;
    if(Math.abs(pidCalculated)< 0.01){
      pidCalculated = 0;
    }
    drive.tankDrive(pidCalculated* 0.5, pidCalculated* 0.5);
    if(isAtPoint() && !alreadyStartedTimer){
      alreadyStartedTimer = true;
      timer.reset();
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }
  
  boolean isAtPoint(){
    double enc = drive.getBothEncoders();
    return enc > setPoint-0.01 && enc < setPoint||
    enc < setPoint+0.01 && enc> setPoint;
   

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(isAtPoint()){
       return true;
    }
    return false;
  }
  
}
