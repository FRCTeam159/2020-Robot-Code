/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.ToggleButton;

public class CameraCommands extends CommandBase {
  /**
   * Creates a new CameraCommands.
   */
  ToggleButton cameraButton = new ToggleButton(Robot.cameraButton);
  Cameras cams;
  boolean prevTrigger = false;

  public CameraCommands(Cameras c) {
    cams = c;
    addRequirements(c);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // boolean cameraButton = Robot.cameraButton.get();
    cams.setBrightness();
    if (cameraButton.newState()) {
      if (Cameras.isFront()) {
        System.out.println("Setting back camera");
        cams.setBack();
      } else {
        System.out.println("Setting front camera");
        cams.setFront();
      }
    }
    // if (cameraButton && !prevTrigger) {
    // if(Cameras.isFront()){
    // System.out.println("Setting back camera");
    // cams.setBack();
    // } else{
    // System.out.println("Setting front camera");
    // cams.setFront();
    // }
    // prevTrigger = true;
    // } else if(prevTrigger && !cameraButton){
    // prevTrigger = false;
    // }
    // cams.writeVideo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
