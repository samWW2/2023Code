package frc.robot.commands.clutch_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClutchSubsystem;


public class OpenCmd extends CommandBase
{
    private ClutchSubsystem clutchSubsystem;
    private  double speed;
  

    public OpenCmd(ClutchSubsystem clutchSubsystem, double speed)
    {
        this.speed = speed;
        this.clutchSubsystem=clutchSubsystem;
        addRequirements(clutchSubsystem);    
    }

    public void execute() 
    {
        clutchSubsystem.setClutchMotor(speed);
           
    }

    @Override
    public void end(boolean interrupted) {
    clutchSubsystem.setClutchMotor(0);  
  }
  public boolean isFinished() {
    
    return false;
  }
}