// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class blncNew extends CommandBase {
  private DriveSubsystem drive;
  private Timer timer;
  private PIDController pid;
  private double kp = -0.019;
  private double feedforward = 0.20;
  

  boolean alreadyStartedTimer = false;
  /** Creates a new blncNew. */
  public blncNew(DriveSubsystem drive) {
    this.drive = drive;
    pid = new PIDController(kp, 0, 0);
    pid.setSetpoint(2);
    pid.setTolerance(5);
    timer = new Timer();
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidCalculated = pid.calculate(drive.getPitch());
    if(pidCalculated >= 0){
      pidCalculated += feedforward;
    }
    else{
      pidCalculated -= feedforward;
    }
    drive.tankDrive(pidCalculated, pidCalculated);

    if (pidCalculated < 0.05 && pidCalculated > -0.05){
    }

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
   if( pid.atSetpoint() && timer.hasElapsed(0.5) ){
    return true;
   }
    return false;
  }
}
