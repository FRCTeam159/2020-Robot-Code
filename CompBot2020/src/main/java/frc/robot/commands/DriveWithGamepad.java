/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ToggleButton;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveWithGamepad extends CommandBase implements Constants {
  double powerScale = 0.45;
  double turnScale = 0.65;
  double moveExponent = 0.75; // Raise moveExponent and turnExponent for more control at lower speeds,
  double turnExponent = 0.75; // and lower them for more control at higher speeds.
  double xMinT = 0.2;
  double xMinO = 0.05;
  double zMinT = 0.05;
  double zMinO = 0;
  boolean useDeadband = true;
  boolean Debug = false;
  ToggleButton shifter = new ToggleButton(Robot.shiftButton);
  private final DriveTrain driveTrain;
  // Button gearButton = new Button(GEAR_BUTTON);

  public DriveWithGamepad(DriveTrain dT) {

    // Use requires() here to declare subsystem dependencies
    driveTrain = dT;
    addRequirements(dT);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    System.out.println("DriveWithGamepad initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // if (!Robot.isTele)
    // return;
    Joystick stick = OI.stick;
    double zs = -stick.getRawAxis(LEFT_JOYSTICK);
    double xs = stick.getRawAxis(RIGHT_JOYSTICK);
    double z = zs;
    double x = xs;

    if(shifter.newState()){
      if (driveTrain.inLowGear())
        driveTrain.setHighGear();
      else
        driveTrain.setLowGear();
    }

    if (useDeadband) {
      z = quadDeadband(zMinT, zMinO, zs);
      x = quadDeadband(xMinT, xMinO, xs);
    }
    double moveAxis = powerScale * z; // left stick - drive
    double turnAxis = powerScale * x; // right stick - rotate
    double moveValue = 0;
    double turnValue = 0;
    if (Debug) {
      System.out.println("xs = " + xs + " x = " + x);
      System.out.println("zs = " + zs + " z = " + z);
    }
    if (Math.abs(moveAxis) > 0.0) {
      moveValue = (moveAxis / Math.abs(moveAxis)) * Math.pow(Math.abs(moveAxis), moveExponent);
    }
    if (Math.abs(turnAxis) > 0.0) {
      turnValue = (turnAxis / Math.abs(turnAxis)) * Math.pow(Math.abs(turnAxis), turnExponent); // Math.abs the
                                                                                                // turnValue
    }

    turnValue *= Math.abs(moveValue) * (1 - turnScale) + turnScale;
    if (!RobotContainer.targeting.isTargeting()) {
      driveTrain.arcadeDrive(moveValue, turnValue);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveWithGamepad end");
  }

  double quadDeadband(double minThreshold, double minOutput, double input) {
    if (input > minThreshold) {
      return ((((1 - minOutput) // 1 - minOutput/(1-minThreshold)^2 * (input-minThreshold)^2 + minOutput
          / ((1 - minThreshold) * (1 - minThreshold))) * ((input - minThreshold) * (input - minThreshold)))
          + minOutput);
    } else {
      if (input < (-minThreshold)) {
        return (((minOutput - 1) // minOutput - 1/(minThreshold - 1)^2 * (minThreshold + input)^2 - minOutput
            / ((minThreshold - 1) * (minThreshold - 1))) * ((minThreshold + input) * (minThreshold + input)))
            - minOutput;
      }

      else {
        return 0;
      }
    }
  }
}
