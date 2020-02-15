/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class ShooterCommands extends CommandBase {
  /**
   * Creates a new ShooterCommands.
   * 
   * @param shooter
   */
  boolean lastState;
  private final Shooter shooter;

  public ShooterCommands(Shooter sT) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = sT;
    addRequirements(sT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightTriggerPressed = OI.stick.getRawAxis(Constants.RIGHT_TRIGGER);
    double leftTriggerPressed = OI.stick.getRawAxis(Constants.LEFT_TRIGGER);
    double leftTrigger = -leftTriggerPressed;
    double rightTrigger = rightTriggerPressed;
    boolean toggleFire = OI.stick.getRawButton(Constants.LEFT_BUMPER_BUTTON);
    boolean feederButton = OI.stick.getRawButton(Constants.RIGHT_BUMPER_BUTTON);
    //method for adjusting robot shooter angle, use triggers.
    if (leftTrigger == 0 && rightTrigger > 0) {
      shooter.changeShootAngle(0.1 * (rightTrigger));
    } else if (rightTrigger == 0 && leftTrigger < 0) {
      shooter.changeShootAngle(0.1 * (leftTrigger));
    }
    //toggle wheel input for launching the balls from shooter, use left bumper.
    if (toggleFire && !lastState) {
      if (shooter.isLaunch()) {
        shooter.setLauch(false);
      } else {
        shooter.setLauch(true);
      }
      lastState = true;
    } else if (!toggleFire && lastState) {
      lastState = false;
    }
    // feeding input for the shooter, use right bumper.
    if (feederButton) {
      shooter.feed(true);
    } else {
      shooter.feed(false);
    }
    shooter.log();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
