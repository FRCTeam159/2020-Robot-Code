/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  public static int count = 1;
  public static int motorValue = 0;
  private SparkMotor shootMotor1;
  private SparkMotor shootMotor2;
  private SparkMotor angleMotor;
  private SparkMotor feedMotor;
  private double shooterMotorValue;
  public boolean launchOn = false;
  public CANDigitalInput forward;
  public CANDigitalInput reverse;

  public Shooter() {
    shooterMotorValue = 1.0;

    if (!RobotContainer.pancake) {
      angleMotor = new SparkMotor(Constants.ANGLE_MOTOR);
      zeroMotor();
      forward = angleMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      reverse = angleMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
      shootMotor1 = new SparkMotor(Constants.SHOOT_MOTOR_R);
      shootMotor2 = new SparkMotor(Constants.SHOOT_MOTOR_L);

      feedMotor = new SparkMotor(Constants.FEED_MOTOR);
      // angle motor must be in break mode instead of coast.

    }
    log();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void changeShootAngle(double value) {
    if (!RobotContainer.pancake) {
      log();
      angleMotor.set(value);
    } else {
      log();
    }
  }

  public void setLauch(boolean launch) {
    if (!RobotContainer.pancake) {
      if (launch) {
        shootMotor1.set(shooterMotorValue);
        shootMotor2.set(-shooterMotorValue);
      } else {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
      }
    }
    launchOn = launch;
  }

  public void feed(boolean isFeed) {
    if (!RobotContainer.pancake) {
      if (isFeed) {
        feedMotor.set(0.4);
      } else {
        feedMotor.set(0.0);
      }
    }
  }

  public boolean isLaunch() {
    return launchOn;
  }

  public boolean isAtTop() {
    return reverse.get();
  }

  public boolean isAtBottom() {
    return forward.get();
  }

  public void log() {
    if (!RobotContainer.pancake) {
      SmartDashboard.putNumber("Angle Rotation", getAngleRotations());
      SmartDashboard.putBoolean("IsAtTop", isAtTop());
      SmartDashboard.putBoolean("IsAtBottom", isAtBottom());
    }
    SmartDashboard.putNumber("Shoot Value", shooterMotorValue);
  }

  public void zeroMotor() {
    if (!RobotContainer.pancake)
      angleMotor.reset();
  }

  public void setLaunchValue(double v) {
    shooterMotorValue = v;
  }

  public double getLaunchValue() {
    return SmartDashboard.getNumber("Shoot Value", 0.0);
  }
  public double getAngleRotations(){
    return angleMotor.getRotations();
  }
}
