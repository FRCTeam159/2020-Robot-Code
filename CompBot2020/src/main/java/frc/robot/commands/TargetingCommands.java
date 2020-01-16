/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Targeting;

public class TargetingCommands extends CommandBase {
  /**
   * Creates a new TargetingCommands.
   */
  NetworkTable table;
  private NetworkTableEntry hAngle;
  private NetworkTableEntry vAngle;
  private NetworkTableEntry distance;
  private NetworkTableEntry haveTarget;
  private final Targeting targeting;


  public TargetingCommands(Targeting tR) {
    targeting = tR;
    addRequirements(tR);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("targetdata");
    SmartDashboard.putNumber("Target distance" , 0.0);
    SmartDashboard.putNumber("H angle" , 0.0);
    SmartDashboard.putNumber("V angle" , 0.0);
    SmartDashboard.putBoolean("Targets", false);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = table.getEntry("tDistance");
    vAngle = table.getEntry("vAngle");
    hAngle = table.getEntry("hAngle");
    haveTarget = table.getEntry("targets");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tdistance = distance.getDouble(0.0);
    double hoff = hAngle.getDouble(0.0);
    double voff = vAngle.getDouble(0.0);
    boolean targets = haveTarget.getBoolean(false);
    SmartDashboard.putNumber("Target distance", round10(tdistance));
    SmartDashboard.putNumber("H angle", round10(hoff));
    SmartDashboard.putNumber("V angle", round10(voff));
    SmartDashboard.putBoolean("Targets", targets);

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

  double round10(double x) {
    return Math.round(x * 10 + 0.5) / 10.0;
  }
}
