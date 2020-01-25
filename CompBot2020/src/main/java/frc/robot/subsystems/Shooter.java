/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public static int count = 1;
  public static int motorValue = 0;
  private SparkMotor shootMotor;
  public Shooter() {
    //shootMotor = new SparkMotor(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void log(){
    count++;
   // System.out.println("IT'S WORKING ANAKIN." + count);

  }
  public double motorCoversion(int rpm){
    //calculate value given to motors from wanted rpm.
    return motorValue;
  }
  public void fire(int firespeed){
    //pass in calculated motorvalue needed to fire.
    //shootMotor.set(firespeed);
  }
}
