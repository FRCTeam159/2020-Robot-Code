/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AdjustTarget extends CommandBase {
  /**
   * Creates a new AdjustTarget.
   */
  
  public AdjustTarget() {
    System.out.println("AdjustTarget.Constructor");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.targeting);
    addRequirements(RobotContainer.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("AdjustTarget.initialize");
    RobotContainer.targeting.enableAutoTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.targeting.doAutoAdjust();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AdjustTarget.end:"+interrupted);
    RobotContainer.targeting.disableAutoTarget();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.targeting.onTarget();
  }
}
