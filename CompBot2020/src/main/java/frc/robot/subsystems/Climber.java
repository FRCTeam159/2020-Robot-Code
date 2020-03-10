/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase implements Constants{
  /**
   * Creates a new Climber.
   */
  SparkMotor climbMotor;
  DoubleSolenoid climbLockPiston;
  DoubleSolenoid climbIntPiston;
  public static boolean climbMode = false;
  public Climber() {
    climbMotor  = new SparkMotor(CLIMB_MOTOR);
    climbLockPiston = new DoubleSolenoid(CLIMBLOCK_PISTON_FORWARD, CLIMBLOCK_PISTON_REVERSE);
    climbIntPiston = new DoubleSolenoid(CLIMBINT_PISTON_FORWARD, CLIMBINT_PISTON_REVERSE);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void initiateClimb(){
    climbIntPiston.set(Value.kReverse);
    climbMode = true;
    
  }
  public void climb(double value){
    if(value > 0.1 || value < 0.1){
      climbLockPiston.set(Value.kForward);
    } else{
      climbLockPiston.set(Value.kReverse);
    }
    climbMotor.set(0.4 * value);
  }
  public boolean isClimbing(){
    return climbMode;
  }
}
