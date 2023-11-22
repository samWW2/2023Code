// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm_commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystemDepract;

public class ArmStill extends CommandBase {
  /** Creates a new Test. */
  private ArmSubsystemDepract arm;
  private ArmFeedforward feedForward;
  private double kg = 0.205;
  public ArmStill(ArmSubsystemDepract arm) {
    feedForward = new ArmFeedforward(0, kg, 0);
    this.arm = arm;
    addRequirements(arm);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltge = feedForward.calculate((arm.getPosition()+90)/360*2*Math.PI, 0);
    arm.setVolt(-voltge);
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
