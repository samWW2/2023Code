package frc.robot.commands.clutch_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClutchSubsystem;


public class CloseCmd extends CommandBase
{
    private ClutchSubsystem clutchSubsystem;
    private  double speed;

    public CloseCmd(ClutchSubsystem clutchSubsystem, double speed)
    {
        this.speed = speed;
        this.clutchSubsystem=clutchSubsystem;
        addRequirements(clutchSubsystem);    
    }

    public void execute() 
    {
        
        clutchSubsystem.setClutchMotor(-speed);        
    }

    @Override
    public void end(boolean interrupted) {
    clutchSubsystem.setClutchMotor(0);  
  }

}