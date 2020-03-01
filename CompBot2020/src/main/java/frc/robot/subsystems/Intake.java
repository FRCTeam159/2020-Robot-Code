/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.repowerIntake;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private SparkMotor hopper;
  private SparkMotor intake;
  private Boolean isHopperEnabled = false;
  private Boolean isIntakeEnabled = false;
  private DoubleSolenoid intakePiston;

  public Intake() {
    intake = new SparkMotor(Constants.INTAKE_MOTOR);
    if (!RobotContainer.pancake) {
      hopper = new SparkMotor(Constants.HOPPER_MOTOR);
      intakePiston = new DoubleSolenoid(Constants.INTAKE_PISTON_FORWARD , Constants.INTAKE_PISTON_REVERSE);
    }
  }

  public void enableHopper(Boolean enable) {
    if (enable) {
      if (!RobotContainer.pancake)
        hopper.set(0.3);
      isHopperEnabled = true;
    } else {
      if (!RobotContainer.pancake)
        hopper.set(0.0);
      isHopperEnabled = false;
    }
  }

  public void enableIntake(Boolean enable) {
    if (enable) {
      if (!RobotContainer.pancake)
        intake.set(-1.0);
      isIntakeEnabled = true;
      System.out.println("intake enabled="+isIntakeEnabled);

    } else {
      if (!RobotContainer.pancake)
        intake.set(0.0);
      isIntakeEnabled = false;
      System.out.println("intake enabled="+isIntakeEnabled);

    }
  }

  public boolean isIntakeEnabled() {
    return isIntakeEnabled;
  }

  public void lowerIntake() {
    if (!RobotContainer.pancake)
      intakePiston.set(Value.kReverse);
  }
  public void repowerIntake(){
    if(!RobotContainer.pancake)
    intakePiston.set(Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
