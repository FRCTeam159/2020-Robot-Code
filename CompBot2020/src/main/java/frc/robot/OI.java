/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements Constants {
  public static final Joystick stick = new Joystick(0);
  static double last_axis[] = new double[8];

  public static void test() {

    for (int i = 0; i < 8; i++) {
      double axis = stick.getRawAxis(i);
      if (axis != last_axis[i])
        System.out.println("Axis [" + i + "] = " + axis);
      last_axis[i] = axis;

    }
  }
}
