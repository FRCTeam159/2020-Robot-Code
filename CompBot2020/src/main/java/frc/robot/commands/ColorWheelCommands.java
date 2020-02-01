/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ColorWheel;

public class ColorWheelCommands extends CommandBase {
  /**
   * Creates a new ColorWheelCommands.
   */
  private final ColorWheel colorWheel;
  public ColorWheelCommands(ColorWheel cW) {
    colorWheel = cW;
    addRequirements(cW);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("ColorWheel", "unknown");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean deployButton = Robot.colorDeployButton.get(); //Left Bumper
    boolean rotationButton = Robot.colorRotationButton.get(); //B button
    boolean matchButton = Robot.colorMatchButton.get(); //X button
    boolean lastState = false;
    if(deployButton && !lastState){
      if(!colorWheel.isDeployed()){
        colorWheel.deploy();
      } else{
        colorWheel.disableColor();
      }
      lastState = true;
    } else if(!deployButton && lastState){
      lastState = false;
    }
    if(rotationButton && colorWheel.isDeployed()){
      colorWheel.doRotations();
    }
    if(matchButton && colorWheel.isDeployed()){
      colorWheel.doColor();
    }
    SmartDashboard.putString("ColorWheel", String.valueOf(colorWheel.getcolorChar()));
    colorWheel.goToState();
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
