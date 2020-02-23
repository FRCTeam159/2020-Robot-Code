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
  private NetworkTableEntry autotarget;
  private NetworkTableEntry hTweek;
  private NetworkTableEntry vTweek;

  public static double hToll = 1.5;
  public static double vToll = 1.5;
  public static double hVMax = 1.0;
  public static double vVMax = 1.0;
  public static double targetHeight = 8.0 + 2.25/12; //center of target height from floor in ft
  public static double radsToDegrees = 360/(2 * Math.PI);
  public double vudge = 0;
  public double wheelDiameter = 4.0 / 12; //ft
  public double wheelCircumference = wheelDiameter * Math.PI; //ft

  NetworkTable table;
  public static boolean autoEnabled = false;
  public static boolean zeroFound = false;
  public static boolean centerFound = false;
  public static boolean targetingDone = false;
  public boolean isVadjust = false;
  Cameras cams;

  public enum states {
    WAITING, FINDINGCENTER, FINDINGOFFSET, ONTARGET
  };

  public states state = states.WAITING;

  /**
   * Creates a new Targeting.
   */
  AdjustH adjustH;
  AdjustV adjustV;

  public Targeting() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("targetdata");
    distance = table.getEntry("tDistance");
    vAngle = table.getEntry("vAngle");
    hAngle = table.getEntry("hAngle");
    haveTarget = table.getEntry("targets");
    atTarget = table.getEntry("atTarget");
    autotarget = table.getEntry("autotarget");
    hTweek = table.getEntry("hTweek");
    vTweek = table.getEntry("vTweek");
    adjustH = new AdjustH();
    adjustV = new AdjustV();
    // SmartDashboard.putNumber("autotarget", 0.0);
  }

  @Override
  public void periodic() {

    // if (autoEnabled) {
    // goToState();
    // }
    // This method will be called once per scheduler run
  }

  public boolean findZero() {
    if (!RobotContainer.shooter.isAtBottom() && !zeroFound) {
      RobotContainer.shooter.changeShootAngle(-0.1);
      zeroFound = false;
    } else {
      RobotContainer.shooter.changeShootAngle(0.0);
      RobotContainer.shooter.zeroMotor();
      zeroFound = true;

    }
    return zeroFound;
  }

  public void goToState() {
    switch (state) {
    default:
    case WAITING:
      centerFound = false;
      targetingDone = false;
      autoEnabled = false;
      break;
    case FINDINGCENTER:
      //System.out.println("we are on target: " + onTarget());
      if (!centerFound) {
        // adjustV.enable();
        // adjustH.enable();
        doAutoAdjust();
      } else {
        centerFound = true;
        state = states.FINDINGOFFSET;
      }
      break;
    case ONTARGET:
       targetingDone = true;
      state = states.WAITING;
      break;
    }
    log();
  }

  public boolean isTargetingDone() {
    return targetingDone;
  }

  public void enableAutoTarget() {
    //if (!autoEnabled) {
      targetingDone = false;
      centerFound=false;
      autoEnabled = true;
      autotarget.setBoolean(autoEnabled);
      state = states.FINDINGCENTER;
      adjustV.enable();
      adjustH.enable();
    //}
    log();
  }
  public void adjustWheelSpeed(){
    double vel = Math.sqrt(targetHeight * 2 * 32); //g = 32ft/sec^2
    double wheelVelocity = vel/wheelCircumference; //ft
  }

  public void log() {
    SmartDashboard.putBoolean("autotarget", autoEnabled);
    SmartDashboard.putBoolean("is Targeting Finished", targetingDone);
  }

  public void disableAutoTarget() {
    state = states.WAITING;
    adjustV.disable();
    adjustH.disable();
    targetingDone = false;
    autoEnabled = false;
    autotarget.setBoolean(autoEnabled);
    log();

  }

  public boolean isTargeting() {
    return autoEnabled;
  }

  public void doAutoAdjust() {
    if (haveTarget.getBoolean(false)) {
      adjustH.calculate();
      adjustV.calculate();
      if(onTarget()){
        centerFound = true;
      }
    }
  }

  public boolean onTarget() {
    if (haveTarget.getBoolean(false)) {
      boolean ht=adjustH.atSetpoint();
      boolean vt=adjustV.atSetpoint();
     // System.out.println("ht = " +  ht + " vt = " + vt);
      
      return ( ht  && vt);
    } else {
      return false;
    }
  }
 
  public double getCurrentAngle(){
    return 0.0;
  }
  public double setFlywheelVelocity(double v){
    return 0.0;
  }

  protected class AdjustH extends PIDController {
    static final double kP = 0.005;
    static final double kI = 0.0;
    static final double kD = 0.0;

    public AdjustH() {
      super(kP, kI, kD, .02);
      setTolerance(hToll, hVMax);
      setSetpoint(0.0);
    }

    public void enable() {
      // System.out.println("enable auto");
      enableContinuousInput(-30, 30);
    }

    public void disable() {
      // .println("disable auto");
      disableContinuousInput();
    }

    public void calculate() {
      double hOff = hAngle.getDouble(0.0);
      double tweek = hTweek.getDouble(0.0);
      double output = super.calculate(hOff + tweek);
     // System.out.println("output is" + output);
      RobotContainer.driveTrain.arcadeDrive(0, -output);
    }
  }

  protected class AdjustV extends PIDController {
    static final double vP = 0.01;
    static final double vI = 0.0;
    static final double vD = 0.0;

    public AdjustV() {
      super(vP, vI, vD, 0.02);
      setTolerance(vToll, vVMax);
      setSetpoint(0.0);
    }

    public void calculate() {
      double vOff = vAngle.getDouble(0.0);
      double tweek = vTweek.getDouble(0.0);
      double output = super.calculate(vOff + tweek);
      RobotContainer.shooter.changeShootAngle(output);
    }

    public void disable() {
      disableContinuousInput();
      if(isVadjust){
      RobotContainer.shooter.changeShootAngle(0.0);
      isVadjust = false;
      }
    }

    public void enable() {
      isVadjust = true;
      enableContinuousInput(-30, 30);
    }
  }
}
