/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private SparkMotor hopper;
  private SparkMotor intake;
  private Boolean isHopperEnabled = false;
  private Boolean isIntakeEnabled = false;
  private Solenoid intakePiston = new Solenoid(Constants.INTAKE_PISTON);
  public Intake() {
    if(!RobotContainer.pancake){
    intake  = new SparkMotor(Constants.INTAKE_MOTOR);
    hopper = new SparkMotor(Constants.HOPPER_MOTOR);
    }
  }
  public void enableHopper(Boolean enable){
    if(enable){
      if(!RobotContainer.pancake)
      hopper.set(0.5);
      isHopperEnabled = true;
    } else{
      if(!RobotContainer.pancake)
      hopper.set(0.0);
      isHopperEnabled = false;
    }
  }
  public void enableIntake(Boolean enable){
    if(enable){
      if(!RobotContainer.pancake)
      intake.set(0.5);
      isIntakeEnabled = true;
    } else{
      if(!RobotContainer.pancake)
      intake.set(0.0);
      isIntakeEnabled = false;
    }
  }
  public boolean isIntakeEnabled(){
    return isIntakeEnabled;
  }
  public void lowerIntake(){
    if(!RobotContainer.pancake)
    intakePiston.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
