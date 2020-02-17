/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase implements Constants {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private SparkMotor frontLeft;
	private SparkMotor frontRight;
	private SparkMotor backLeft;
	private SparkMotor backRight;
	private static final double WHEEL_DIAMETER = 4.25; // in// 8 i-n wheels on 2019bot
	private static final double MEASURED_FEET_PER_REV = (10 / 134.0);
	private static final double LOW_GEARBOX_RATIO = 8.333;
	private static final double HIGH_GEARBOX_RATIO = 3.667;
	private static final double DRIVE_SPROCKET_RATIO = 38/22;
	private static final double LOW_FINAL_GEAR_RATIO = LOW_GEARBOX_RATIO * DRIVE_SPROCKET_RATIO;
	private static final double HIGH_FINAL_GEAR_RATIO = HIGH_GEARBOX_RATIO * DRIVE_SPROCKET_RATIO;
	public static  double TRUE_GEAR_RATIO;
	private static final double CALC_FEET_PER_REV = Math.PI * WHEEL_DIAMETER * LOW_FINAL_GEAR_RATIO / 12;
	private static final double INCHES_PER_REV = MEASURED_FEET_PER_REV * 12;
	private static final double CALC_INCHES_PER_REV = CALC_FEET_PER_REV * 12;
	
	private ADXRS450_Gyro gyro;
	//private DoubleSolenoid gearPneumatic;
	public boolean lowGear = false;

	public DriveTrain() {
		frontLeft = new SparkMotor(FRONT_LEFT);
		frontRight = new SparkMotor(FRONT_RIGHT);
		backLeft = new SparkMotor(BACK_LEFT);
		backRight = new SparkMotor(BACK_RIGHT);
		gyro = new ADXRS450_Gyro();
		//gearPneumatic = new DoubleSolenoid(RobotMap.GEAR_SHIFTER_FORWARD, RobotMap.GEAR_SHIFTER_REVERSE);

		System.out.println("Gear ratio is: " + LOW_FINAL_GEAR_RATIO);

		/*
		 * frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
		 * RobotMap.TIMEOUT);
		 * backLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
		 * RobotMap.TIMEOUT); System.out.println("Ticks Per Foot: " + TICKS_PER_FOOT);
		 * gyro = new ADXRS450_Gyro();
		 */
		// frontLeft.setSafetyEnabled(true);
		reset();
	}

	public void arcadeDrive(double moveValue, double turnValue) {
		double leftMotorOutput;
		double rightMotorOutput;
		//System.out.println("M:"+moveValue+" T:"+turnValue);

		if (moveValue > 0.0) {
			if (turnValue > 0.0) {
				leftMotorOutput = Math.max(moveValue, turnValue);
				rightMotorOutput = moveValue - turnValue;
			} else {
				leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = Math.max(moveValue, -turnValue);
			}
		} else {
			if (turnValue > 0.0) {
				leftMotorOutput = moveValue + turnValue;
				rightMotorOutput = -Math.max(-moveValue, turnValue);
			} else {
				leftMotorOutput = -Math.max(-moveValue, -turnValue);
				rightMotorOutput = moveValue - turnValue;
			}
		}
		// Make sure values are between -1 and 1
		leftMotorOutput = coerce(-1, 1, leftMotorOutput);
		rightMotorOutput = coerce(-1, 1, rightMotorOutput);
		/*
		 * System.out.printf("l:%d r:%d\n",
		 * frontRight.getSensorCollection().getQuadraturePosition(),
		 * -backLeft.getSensorCollection().getQuadraturePosition());
		 */
		// lastMoveValue = moveValue;
		setRaw(leftMotorOutput, rightMotorOutput);
		// setRaw(0, rightMotorOutput);
	}

	private static double coerce(double min, double max, double value) {
		return Math.max(min, Math.min(value, max));
	}

	public void setRaw(double left, double right) {
		backLeft.set(left);
		frontLeft.set(left);
		frontRight.set(-right);
		backRight.set(-right);
		log();
	}

	public void reset() {
		gyro.reset();
		backLeft.reset();
		backRight.reset();
		frontLeft.reset();
		frontRight.reset();
		setLowGear();
		log();

	}

	public void enable() {
		log();
	}

	public void disable() {

	}

	public double getDistance() {
		double d1 = getRightDistance();
		double d2 = getLeftDistance();
		return 0.5 * (d1 + d2);
	}

	// both return values in feet, number of rotations are averaged between the two
	// motors for each side.
	public double getRightDistance() {
		double rightPosition = backRight.getRotations() + frontRight.getRotations();
		return -CALC_INCHES_PER_REV * rightPosition / 2;
		// return frontRight.getSensorCollection().getQuadraturePosition() /
		// TICKS_PER_FOOT;
	}

	public double getLeftDistance() {
		double leftPosition = backLeft.getRotations() + frontLeft.getRotations();
		return CALC_INCHES_PER_REV * leftPosition / 2;

		// return -backLeft.getSensorCollection().getQuadraturePosition() /
		// TICKS_PER_FOOT;
	}
	private double getRevolutions(){
		double leftRevs =  0.5*(backLeft.getRotations() + frontLeft.getRotations());
		double rightRevs = -0.5*(backRight.getRotations() + frontRight.getRotations());
		return 0.5*(leftRevs + rightRevs);
	}

	private void log() {
		 SmartDashboard.putNumber("Heading", getHeading());
		// SmartDashboard.putNumber("Left wheels",
		// -backLeft.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Right wheels",
		// frontRight.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Left distance", getLeftDistance());
		// SmartDashboard.putNumber("Right distance", getRightDistance());
		// SmartDashboard.putNumber("Velocity", getVelocity());
		SmartDashboard.putNumber("Revolutions", getRevolutions());
		SmartDashboard.putNumber("Distance", getDistance());
		// SmartDashboard.putBoolean("Low Gear", inLowGear());
	}

	public double getHeading() {
		double angle = gyro.getAngle() % 360;
		if(angle < -180){
			angle += 360;
		} else if (angle > 180 ){
			angle -= 360;
		}
		return angle;
	
	}

	public void setHighGear() {
		if (lowGear) {
	//		gearPneumatic.set(DoubleSolenoid.Value.kReverse);
			lowGear = false;
			TRUE_GEAR_RATIO = HIGH_FINAL_GEAR_RATIO;
		}
	}

	public void setLowGear() {
		if (!lowGear) {
		//	gearPneumatic.set(DoubleSolenoid.Value.kForward);
			lowGear = true;
			TRUE_GEAR_RATIO = LOW_FINAL_GEAR_RATIO;
		}
	}

	public boolean inLowGear() {
		return lowGear;

	}
}
