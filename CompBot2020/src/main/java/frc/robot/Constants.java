/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {


     // Motor IDs
  public static final int FRONT_LEFT = 3;
  public static final int BACK_LEFT = 4;

  public static final int FRONT_RIGHT = 2;
  public static final int BACK_RIGHT = 1;

  public static final int COLOR_MOTOR = 11;
  public static final int HOPPER_MOTOR = 12;
  public static final int FEED_MOTOR = 10;
  public static final int ANGLE_MOTOR = 9;
  public static final int SHOOT_MOTOR_R = 5;
  public static final int SHOOT_MOTOR_L = 6;
  public static final int INTAKE_MOTOR = 8;
  public static final int CLIMB_MOTOR = 7;
  // Piston IDs
  public static final int GEAR_SHIFTER_FORWARD = 0;
  public static final int GEAR_SHIFTER_REVERSE = 1;
  public static final int COLOR_PISTON_FORWARD = 2;
  public static final int COLOR_PISTON_REVERSE = 3;
  public static final int INTAKE_PISTON_FORWARD = 2;
  public static final int INTAKE_PISTON_REVERSE = 3;

  // Servo IDs
  public static final int ARM_SERVO = 0;
  // Controller IDs
  public static final int DRIVER = 0;
  // These are button IDs
  public static final int LEFT_JOYSTICK = 1;
  public static final int RIGHT_JOYSTICK = 4;
  public static final int LEFT_TRIGGER = 2;
  public static final int RIGHT_TRIGGER = 3;
  public static final int LEFT_BUMPER_BUTTON = 5;
  public static final int RIGHT_BUMPER_BUTTON = 6;
  public static final int B_BUTTON = 2;
  public static final int A_BUTTON = 1;
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;

  //public static final int GEAR_BUTTON = 7;
  public static final int START_BUTTON = 8;
  public static final int BACK_BUTTON = 7;
  public static final int LEFT_JOYSTICK_BUTTON = 9;
  public static final int RIGHT_JOYSTICK_BUTTON = 10;
  // General Constants
  public static final int TIMEOUT = 10;
  public static final int ENCODER_TIMEOUT = 10;
  public static final int ENCODER_WINDOW_SIZE = 4;
  public static final int ENCODER_STATUS_FRAME_PERIOD = 4;
  public static final int SHOOTER_ENCODER_RATIO = 10; //10:1 gear ratio

  public static final int LEFT_POSITION = 0;
  public static final int CENTER_POSITION = 1;
  public static final int RIGHT_POSITION = 2;
  public static final int ILLEGAL_POSITION = 3;

  // Physical Dimensions

  public static double radsToDegrees = 360/(2 * Math.PI);

  public static double launcherWheelDiameter = 4.0 / 12; //ft
  public static double launcherWheelCircumference = launcherWheelDiameter * Math.PI; //ft
  public static double targetWidth = 34.0; // physical width of target (inches)
  // ht of target center from floor
  public static double targetFloorHeight = 8 * 12 + 2.25; // actual field target ht
  //public static double targetFloorHeight = 38.0; // test target height
  public static double cameraFloorHeight =16.0; 
  public static double targetHeight = targetFloorHeight-cameraFloorHeight; //center of target height from floor in ft

  static void configure(){
  }
}
