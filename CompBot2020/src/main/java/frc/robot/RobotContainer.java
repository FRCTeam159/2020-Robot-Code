/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static boolean pancake = true;

  public static final DriveTrain driveTrain = new DriveTrain();
  private final DriveWithGamepad driveWithGamepad = new DriveWithGamepad(driveTrain);
  public static final Shooter shooter = new Shooter();
  private final ShooterCommands shooterCommands = new ShooterCommands(shooter);
  public static final Intake intake = new Intake();
  public static IntakeCommands intakeCommands = new IntakeCommands(intake);
  public static final Climber climber = new Climber();
  private final ClimberCommands climberCommands = new ClimberCommands(climber);
  public static final ColorWheel colorWheel = new ColorWheel();
  private final ColorWheelCommands colorWheelCommands = new ColorWheelCommands(colorWheel);
  public static final Cameras cameras = new Cameras();
  private final CameraCommands cameraCommands =  new CameraCommands(cameras);
  public static final Targeting targeting = new Targeting();
  private final TargetingCommands targetingCommands =  new TargetingCommands(targeting);

  private final VisionProcess vision = new VisionProcess();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    vision.init();
    vision.start();
    driveTrain.setDefaultCommand(driveWithGamepad);
    shooter.setDefaultCommand(shooterCommands);
    targeting.setDefaultCommand(targetingCommands);
    colorWheel.setDefaultCommand(colorWheelCommands);
    intake.setDefaultCommand(intakeCommands);
    climber.setDefaultCommand(climberCommands);
    cameras.setDefaultCommand(cameraCommands);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutonomousCommand();
  }
}
