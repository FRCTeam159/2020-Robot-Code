/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Targeting extends SubsystemBase {
  private NetworkTableEntry hAngle;
  private NetworkTableEntry vAngle;
  private NetworkTableEntry distance;
  private NetworkTableEntry haveTarget;
  private NetworkTableEntry atTarget;
  NetworkTable table;
  public static boolean autoEnabled = false;

  /**
   * Creates a new Targeting.
   */
  AdjustH adjustH;

  public Targeting() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("targetdata");
    distance = table.getEntry("tDistance");
    vAngle = table.getEntry("vAngle");
    hAngle = table.getEntry("hAngle");
    haveTarget = table.getEntry("targets");
    atTarget = table.getEntry("atTarget");
    adjustH = new AdjustH();
    SmartDashboard.putNumber("targeting", 0.0);
  }

  @Override
  public void periodic() {
    if (autoEnabled) {
      doAutoAdjust();
    }
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Y button", Robot.targetButton.get());
  }

  public void enableAutoTarget() {
    if (!(autoEnabled)) {
      adjustH.enable();
      autoEnabled = true;
    }
  }

  public void disableAutoTarget() {
    adjustH.disable();
    autoEnabled = false;
  }

  public boolean isTargeting() {
    return autoEnabled;
  }

  public void doAutoAdjust() {
    adjustH.calculate();
  }

  public boolean onTarget() {
    return adjustH.atSetpoint();
  }

  protected class AdjustH extends PIDController {
    static final double kP = 0.01;
    static final double kI = 0.0;
    static final double kD = 0.0;

    public AdjustH() {
      super(kP, kI, kD, .02);
      setTolerance(0.5, 1);
      setSetpoint(0.0);
    }

    public void enable() {
      //System.out.println("enable auto");
      enableContinuousInput(-30, 30);
    }

    public void disable() {
      //.println("disable auto");
      disableContinuousInput();
    }

    public void calculate() {
      double hoff = hAngle.getDouble(0.0);
      double output = super.calculate(hoff);
      SmartDashboard.putNumber("targeting", output);
      RobotContainer.driveTrain.arcadeDrive(0, -output);
    }
  }
}
