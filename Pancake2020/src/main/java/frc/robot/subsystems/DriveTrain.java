/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithGamepad;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import frc.robot.subsystems.SparkMotor;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem implements RobotMap {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private SparkMotor frontLeft;
	private SparkMotor frontRight;
	private SparkMotor backLeft;
	private SparkMotor backRight;
	private static final double WHEEL_DIAMETER = 8; // in// 8 in wheels on 2019bot
	private static final double WHEEL_FEET_PER_REV = Math.PI * 8 / 12;
	private static final double MEASURED_FEET_PER_REV = 10 / 18.8;
	private static final double GEAR_RATIO = WHEEL_FEET_PER_REV / MEASURED_FEET_PER_REV;
	private static final double INCHES_PER_REV = MEASURED_FEET_PER_REV * 12;
	//private ADXRS450_Gyro gyro;
	private DoubleSolenoid gearPneumatic;
	public boolean lowGear = false;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithGamepad());
	}

	public DriveTrain() {
		frontLeft = new SparkMotor(FRONT_LEFT);
		frontRight = new SparkMotor(FRONT_RIGHT);
		backLeft = new SparkMotor(BACK_LEFT);
		backRight = new SparkMotor(BACK_RIGHT);
		//gyro = new ADXRS450_Gyro();
		gearPneumatic = new DoubleSolenoid(RobotMap.GEAR_SHIFTER_FORWARD, RobotMap.GEAR_SHIFTER_REVERSE);

		System.out.println("Gear ratio is: " + GEAR_RATIO);

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
	//	gyrogyro.reset();
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
		return -INCHES_PER_REV * rightPosition / 2;
		// return frontRight.getSensorCollection().getQuadraturePosition() /
		// TICKS_PER_FOOT;
	}

	public double getLeftDistance() {
		double leftPosition = backLeft.getRotations() + frontLeft.getRotations();
		return INCHES_PER_REV * leftPosition / 2;

		// return -backLeft.getSensorCollection().getQuadraturePosition() /
		// TICKS_PER_FOOT;
	}

	private void log() {
		// SmartDashboard.putNumber("Heading", getHeading());
		// SmartDashboard.putNumber("Left wheels",
		// -backLeft.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Right wheels",
		// frontRight.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Left distance", getLeftDistance());
		// SmartDashboard.putNumber("Right distance", getRightDistance());
		// SmartDashboard.putNumber("Velocity", getVelocity());
		SmartDashboard.putNumber("Distance", getDistance());
		SmartDashboard.putBoolean("Low Gear", inLowGear());
	}

	public double getHeading() {
		//return gyro.getAngle();
		return 0;
	}

	public void setHighGear() {
		if (lowGear) {
			gearPneumatic.set(DoubleSolenoid.Value.kReverse);
			lowGear = false;
		}
	}

	public void setLowGear() {
		if (!lowGear) {
			gearPneumatic.set(DoubleSolenoid.Value.kForward);
			lowGear = true;
		}
	}

	public boolean inLowGear() {
		return lowGear;

	}
}
