package frc.robot.commands.drive_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DriveSubsystem;


public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final DriveSubsystem _driveTrain; // declares drive train
  private final Joystick _leftJoystick; // declares left joystick
  private final Joystick _rightJoystick; //declares rightjoystick

  public TankDrive(DriveSubsystem dt, Joystick leftJ, Joystick rightJ) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt; //initializes drive train
    _leftJoystick = leftJ; // initializes left joystick
    _rightJoystick = rightJ; // initializes right joystick

    addRequirements(_driveTrain); // tank drive has to use drive train
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    _driveTrain.tankDrive(0.8 * _leftJoystick.getRawAxis(1), // gets y axis from joysticks and moves accordingly 
                          0.8 * _rightJoystick.getRawAxis(1));  
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
