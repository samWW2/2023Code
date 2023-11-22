

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CenterToTarget;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TestCommand;
import frc.robot.commands.arm_commands.ArmStill;
import frc.robot.commands.arm_commands.ArmStillAuto;
import frc.robot.commands.arm_commands.armArcadeLiftCmd;
import frc.robot.commands.arm_commands.autoLift;
import frc.robot.commands.arm_commands.down;
import frc.robot.commands.clutch_commands.CloseCmd;
import frc.robot.commands.clutch_commands.OpenCmd;
import frc.robot.commands.clutch_commands.OpenCmdAuto;
import frc.robot.commands.drive_commands.ArcadeDriveCmd;
import frc.robot.commands.drive_commands.DriveToDis;
import frc.robot.commands.drive_commands.DriveToDisBack;
import frc.robot.commands.drive_commands.TurnToDeg;
import frc.robot.commands.drive_commands.blncNew;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystemDepract;
import frc.robot.subsystems.ClutchSubsystem;
import frc.robot.subsystems.DriveSubsystem;



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick arm_stick = new Joystick(0);
  private final GenericHID joy = new GenericHID(1);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArmSubsystemDepract armSubsystemDepract = new ArmSubsystemDepract();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ClutchSubsystem Clutch = new ClutchSubsystem();
  private  static LimeLight lime = new LimeLight();
  private Command defaultt = new ParallelCommandGroup(
      new ArmStillAuto(armSubsystemDepract, 8),
      new DriveToDis(driveSubsystem, 0.6),
      new SequentialCommandGroup(
        new WaitCommand(1),
         new OpenCmdAuto(Clutch, 0.35, 5),                          
         new WaitCommand(1)
      )
    );
  private Command side =  new SequentialCommandGroup(
    new autoLift(armSubsystemDepract),
     new  ParallelCommandGroup(
      new ArmStillAuto(armSubsystemDepract, 8),
      new DriveToDis(driveSubsystem, 0.6),
      new SequentialCommandGroup(
        new WaitCommand(1),
         new OpenCmdAuto(Clutch, 0.35, 5),                          
         new WaitCommand(1)
      )
    ),
      new ParallelCommandGroup(
        new DriveToDisBack(driveSubsystem, 3.8),
        new ArmStill (armSubsystemDepract)
      )
    );
  private Command Middle = new SequentialCommandGroup(
    new ParallelCommandGroup(
      new ArmStillAuto(armSubsystemDepract, 8),
      new DriveToDis(driveSubsystem, 0.6),
      new SequentialCommandGroup(
        new WaitCommand(1),
         new OpenCmdAuto(Clutch, 0.35, 5),                          
         new WaitCommand(0.5)
      )
    ),
    new DriveToDisBack(driveSubsystem, 1.30),
    new blncNew(driveSubsystem)
    );
    // to side add go back
    // to middle add go back and on ramp
  SendableChooser<Command> m_Chooser = new SendableChooser<>();





  public RobotContainer() {
    double pitch1 = driveSubsystem.getPitch();
     SmartDashboard.putNumber("pitch", pitch1);
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> -joy.getRawAxis(2), () -> joy.getRawAxis(4)));
    armSubsystemDepract.setDefaultCommand(new armArcadeLiftCmd(armSubsystemDepract, () -> arm_stick.getRawAxis(1)));
    
    m_Chooser.setDefaultOption("default auto", defaultt);
    m_Chooser.addOption("side auto", side);
    m_Chooser.addOption("middle auto", Middle);
    SmartDashboard.putData(m_Chooser);
    
 
    configureBindings();
  }

 
  private void configureBindings() {
    new JoystickButton(arm_stick, 5).whileTrue(new OpenCmd(Clutch,0.3));//open
    new JoystickButton(arm_stick, 6).whileTrue(new CloseCmd(Clutch, 0.45));//close
    new JoystickButton(arm_stick, 3).whileTrue(new OpenCmd(Clutch,0.2));//open soft
    new JoystickButton(arm_stick, 4).whileTrue(new CloseCmd(Clutch,0.2));//close soft
    new JoystickButton(arm_stick, 2).whileTrue(new down(armSubsystemDepract));//make arm go down
    new JoystickButton(arm_stick, 12).whileTrue(new ArmStill(armSubsystemDepract));//make arm Still
    // new JoystickButton(joy, 1).whileTrue(new SequentialCommandGroup(new autoLift(armSubsystemDepract), new ArmStill(armSubsystemDepract)));

 
  // new JoystickButton(joy, 1).onTrue(new TurnToDeg(driveSubsystem, 180));
  // JoystickButton toTarget = new JoystickButton(joy, 3);
  //   toTarget.onTrue( new CenterToTarget(lime, driveSubsystem));

  
  }

  
  //  public SequentialCommandGroup getAutonomousCommand() {
  //   return  new SequentialCommandGroup(new DriveToDis(driveSubsystem, 1.50), new blncNew(driveSubsystem));
  // }
  //  public Command getAutonomousCommand() {
  //   return  new SequentialCommandGroup(
  //     new autoLift(armSubsystemDepract),
  //      new  ParallelCommandGroup(
  //       new ArmStillAuto(armSubsystemDepract, 8),
  //       new DriveToDis(driveSubsystem, 0.2), // was 0.6
  //       new SequentialCommandGroup(
  //         new WaitCommand(1),
  //          new OpenCmdAuto(Clutch, 0.35, 5),                          
  //          new WaitCommand(1)
  //       )
  //     ));
  //   }
  //  public Command getAutonomousCommand() {
  //   return new blncNew(driveSubsystem);
  // }
  // public Command getAutonomousCommand() {
   
  //   return m_Chooser.getSelected();
  // }
   public Command getAutonomousCommand() {
   
    return null;
  }
  //   public Command getAutonomousCommand() {
   
  //   return new DriveToDis(driveSubsystem, 1);
  // }
  // public SequentialCommandGroup getAutonomousCommand() {
   
  //   return  new SequentialCommandGroup(
  //     new autoLift(armSubsystemDepract),
  //      new  ParallelCommandGroup(
  //       new ArmStillAuto(armSubsystemDepract, 8),
  //       new DriveToDis(driveSubsystem, 0.6), // was 0.6
  //       new SequentialCommandGroup(
  //         new WaitCommand(1),
  //          new OpenCmdAuto(Clutch, 0.35, 5),                          
  //          new WaitCommand(1)
  //       )
  //     ),
  //       new ParallelCommandGroup(
  //         new DriveToDisBack(driveSubsystem, 2),
  //         new ArmStill (armSubsystemDepract)
  //       )
  //     );
  // }

  public void onAuto() {
    driveSubsystem.setIdleMode(IdleMode.kBrake);
  }

  public void onTeleop() {
    driveSubsystem.setIdleMode(IdleMode.kCoast);
  }
  public static double get_tv() {
    return 0;
  }

public static double get_ty() {
    return 0;
}

} 
