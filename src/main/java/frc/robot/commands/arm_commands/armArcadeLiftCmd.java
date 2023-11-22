package frc.robot.commands.arm_commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystemDepract;
import frc.robot.subsystems.DriveSubsystem;

public class armArcadeLiftCmd extends CommandBase {

    private final ArmSubsystemDepract armSubsystemDepract;
    private final Supplier<Double> speedFunction; 

    public armArcadeLiftCmd(ArmSubsystemDepract armSubsystemDepract, Supplier<Double> speedFunction)
    {
        this.speedFunction = speedFunction;
        this.armSubsystemDepract = armSubsystemDepract;

        addRequirements(armSubsystemDepract);
      
    }

    @Override
    public void initialize() {
        armSubsystemDepract.resetEncoders();
    }

    @Override
    public void execute() {
        double realTimeSpeed = Math.abs(speedFunction.get());
        
        if(realTimeSpeed < 0.2)
        {
            realTimeSpeed = 0;
        }
      
        
        
        armSubsystemDepract.setMotor(realTimeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}