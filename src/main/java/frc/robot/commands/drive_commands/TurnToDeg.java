// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive_commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.SPI; // this import


public class TurnToDeg extends CommandBase {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final DriveSubsystem drive;
  private final double kp = 0.1;
  private double speed;
  private double angle;

  /** Creates a new TurnTo180. */
  public TurnToDeg(DriveSubsystem drive, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   double error = gyro.getAngle(); // chnage to x axis
   speed = error * kp;
  if(gyro.getAngle() == angle){
    drive.arcadeDrive(0,0);
  }
  else{
    drive.arcadeDrive(0, speed);

  }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
