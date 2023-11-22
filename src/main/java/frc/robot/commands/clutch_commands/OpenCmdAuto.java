package frc.robot.commands.clutch_commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClutchSubsystem;


public class OpenCmdAuto extends CommandBase
{
    private ClutchSubsystem clutchSubsystem;
    private  double speed;

    private Timer timer;
    private double period;

  

    public OpenCmdAuto(ClutchSubsystem clutchSubsystem, double speed, double period)
    {
        this.speed = speed;
        this.period = period;
        this.clutchSubsystem=clutchSubsystem;
        addRequirements(clutchSubsystem);
        timer = new Timer();
        timer.reset();
        timer.start();
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
    return timer.hasElapsed(period);
  }
}