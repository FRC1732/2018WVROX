package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.Robot;
//import org.omg.CORBA.TRANSACTION_UNAVAILABLE;
import org.usfirst.frc.team1732.robot.RobotMap;
import org.usfirst.frc.team1732.robot.commands.DriveWithArcade;
import org.usfirst.frc.team1732.robot.commands.DriveWithJoysticks;
import org.usfirst.frc.team1732.robot.commands.TurnToAngle;

//import com.ctre.MotorControl.SmartMotorController.TalonControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public static final String NAME = "Drive Train";
	private TurnToAngle angleControl;

	// CONTROL SHIFT F IS YOUR FRIEND

	public DriveTrain() {
		super(NAME);
		configureTalons();
		configureEncoders();
		angleControl = new TurnToAngle(0, false);
	}

	private final double INCHES_PER_PULSE = 1.0 / 120;
	private final double ROBOT_DIAMETER = 26;
	
	private static final double LEFT_SPEED = 0.9;
	private static final double RIGHT_SPEED = 1.0;

	// Motor Declaration
	//private TalonSRX leftMaster;
	//private TalonSRX rightMaster;
	
	private final TalonSRX leftMaster = new TalonSRX(RobotMap.LEFT_MASTER_MOTOR_DEVICE_NUMBER);
	private final TalonSRX left1 = new TalonSRX(RobotMap.LEFT_1_MOTOR_DEVICE_NUMBER);
	private final TalonSRX left2 = new TalonSRX(RobotMap.LEFT_2_MOTOR_DEVICE_NUMBER);
	private final TalonSRX rightMaster = new TalonSRX(RobotMap.RIGHT_MASTER_MOTOR_DEVICE_NUMBER);
	private final TalonSRX right1 = new TalonSRX(RobotMap.RIGHT_1_MOTOR_DEVICE_NUMBER);
	private final TalonSRX right2 = new TalonSRX(RobotMap.RIGHT_2_MOTOR_DEVICE_NUMBER);


	// Encoder Declaration & Variables
	public final Encoder leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_CHANNEL_A, RobotMap.LEFT_ENCODER_CHANNEL_B);
	public final Encoder rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_CHANNEL_A,
			RobotMap.RIGHT_ENCODER_CHANNEL_B);
	public static final double INCHES_PER_ENCODER_COUNT = 44 / 5425.4;

	private final PIDController leftEncoderPID = new PIDController(encoderP, encoderI, encoderD, leftEncoder,
			DriveTrain::voidMethod);// , 20);
	private final PIDController rightEncoderPID = new PIDController(encoderP, encoderI, encoderD, rightEncoder,
			DriveTrain::voidMethod);// , 20);

	public static final double encoderP = 0.03; // 0.02
	public static final double encoderI = 0;
	public static final double encoderD = 0;
    public static final double ENCODER_DEADBAND_INCHES = 3; // 6
	public static final double errorDifferenceScalar = 0.045; // 0.035

	public static final double ENCODER_IZONE = 20;
	public static final double ENCODER_IZONE_I = 0.0004;

	public static final double RIGHT_PERCENTAGE_FORWARD = 1;// 0.968;
	public static final double RIGHT_PERCENTAGE_BACKWARD = 1;
	public static final double LEFT_PERCENTAGE_FORWARD = 1;// 0.978;
	public static final double LEFT_PERCENTAGE_BACKWARD = 1;

	private void configureTalons() {		
		leftMaster.setInverted(true);
		left1.setInverted(true);
		left2.setInverted(true);
		rightMaster.setInverted(false);

		left1.set(ControlMode.Follower, leftMaster.getDeviceID());
		left2.set(ControlMode.Follower, leftMaster.getDeviceID());
		right1.set(ControlMode.Follower, rightMaster.getDeviceID());
		right2.set(ControlMode.Follower, rightMaster.getDeviceID());

		leftMaster.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);
		
		//leftMaster = Robot.config.talon("drivetrain.left");
		//rightMaster = Robot.config.talon("drivetrain.right");
		
		
	}

	private void configureEncoders() {
		leftEncoder.setDistancePerPulse(INCHES_PER_PULSE);
		rightEncoder.setDistancePerPulse(INCHES_PER_PULSE);
		leftEncoder.setReverseDirection(true);
		rightEncoder.setReverseDirection(false);
		leftEncoderPID.setAbsoluteTolerance(ENCODER_DEADBAND_INCHES);
		rightEncoderPID.setAbsoluteTolerance(ENCODER_DEADBAND_INCHES);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoysticks());
		//setDefaultCommand(new DriveWithArcade());
		// setDefaultCommand(new DriveWithStick());
	}
	
	public void periodic() {
		System.out.println("Left Encoder Position" + leftEncoder.getDistance());
		System.out.println("Right Encoder Position" + rightEncoder.getDistance());
		
		SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getDistance());
	}

	private static final double PRECISION = 10;
	// using 50% speed
	private static final double multiplier = 0.7;

	public void driveWithJoysticks(double left, double right) {
		rightMaster.set(ControlMode.PercentOutput, multiplier * left);
		// Go forward on the left side as much the left throttle is pushed
		// upwards

		leftMaster.set(ControlMode.PercentOutput, multiplier * right);
		// Go forward on the right side as much the right throttle is pushed
		// upwards
	}

	public void driveWithArcade(double turn, double throttle) {
		turn = turn * -1;
		leftMaster.set(ControlMode.PercentOutput, multiplier * (throttle + turn));
		rightMaster.set(ControlMode.PercentOutput, multiplier * (throttle - turn));
		// I'll be real, I don't even know
	}

	private boolean close = false;

	public void driveWithStick(double x, double y, double angle) {
		x = -x; // Reverses x to be desireable
		double destinationAngle = Math.atan(y / x); // finds angle to turn to
		if (destinationAngle < 0) { // Handles negative angles by making them
									// positive
			destinationAngle = Math.PI * 2 + destinationAngle;
		}

		if (x < 0) { // Account for negative x (domain of arctangent)
			destinationAngle += Math.PI;
		}
		if (Math.abs(x) < 0.1) { // If x is about 0, you're close (when x is not
									// active, stay still)
			close = true;
		}
		destinationAngle = Math.toDegrees(destinationAngle) % 360;
		/*
		 * Converts to degrees + restricts to 360 degrees via mod
		 */
		SmartDashboard.putNumber("DestAngle", destinationAngle);
		double speed = -Math.sqrt(x * x + y * y); // Finds hypotenuse of
													// triangle

		// if not close, then run turn to angle
		if (!close) {
			angleControl.setAngle(destinationAngle); // Sets angle to turn to
			angleControl.runSync(); // Executes TurnToAngle
			// if angleControl hit target, close = true
			if (angleControl.isDone()) { // Checks if done
				close = true;
			}

		} else {
			// if close, recheck close, and run forward
			close = Math.abs(destinationAngle - angle) <= PRECISION;
			// Go forward at the speed of the hypotenuse of the triangle
			leftMaster.set(ControlMode.PercentOutput, multiplier * speed);
			rightMaster.set(ControlMode.PercentOutput, multiplier * speed);
		}
	}

	/*
	 * AUTON BEGINS BELOW HERE
	 */

	// AUTON Pommes
	private static final double autoSpped = 100;

	public void rotate(double d) {
		leftMaster.set(ControlMode.PercentOutput, -Math.min(d / autoSpped, 0.5));
		rightMaster.set(ControlMode.PercentOutput, Math.min(d / autoSpped, 0.5));
	}

	// AUTON Visaya
	public void rotateCW(double speed) {
		// Left is inverted
		leftMaster.set(ControlMode.PercentOutput, -speed);
		rightMaster.set(ControlMode.PercentOutput, speed);
	}

	public void rotateCCW(double speed) {
		// Right is inverted
		leftMaster.set(ControlMode.PercentOutput, speed);
		rightMaster.set(ControlMode.PercentOutput, -speed);
	}

	public void drive(double speed) {
		leftMaster.set(ControlMode.PercentOutput, multiplier * speed);
		rightMaster.set(ControlMode.PercentOutput, multiplier * speed);
	}

	public void driveIndependant(double leftSpeed, double rightSpeed) {
		/*
		System.out.println("Left Speed: " + leftSpeed);
		System.out.println("Right Speed: " + rightSpeed);
		*/
		rightMaster.set(ControlMode.PercentOutput, multiplier * leftSpeed * LEFT_SPEED);
		leftMaster.set(ControlMode.PercentOutput, multiplier * rightSpeed * RIGHT_SPEED);
	}

	public void stop() {
		leftMaster.set(ControlMode.PercentOutput, 0);
		rightMaster.set(ControlMode.PercentOutput, 0);
	}
	
	public double getMultiplier() {
		return multiplier;
	}

	public double getLeftEncoder() {
		return leftEncoder.getDistance();
	}

	public double getRightEncoder() {
		return rightEncoder.getDistance();
	}

	public double getRotateCircumference() {
		return ROBOT_DIAMETER * Math.PI;
	}

	public double getRobotRadius() {
		return ROBOT_DIAMETER / 2;
	}

	public double getLeftPIDError() {
		return leftEncoderPID.getError();
	}

	public double getRightPIDError() {
		return rightEncoderPID.getError();
	}

	public void resetEncoderPIDValues() {
		leftEncoderPID.setPID(encoderP, encoderI, encoderD);
		rightEncoderPID.setPID(encoderP, encoderI, encoderD);
	}
	
	public void setLeftEncoderSetpoint(double setpoint) {
		leftEncoderPID.setSetpoint(setpoint);
	}
	
	public void setRightEncoderSetpoint(double setpoint) {
		rightEncoderPID.setSetpoint(setpoint);
	}
	
	public double getLeftAdjustment() {
		return leftEncoderPID.getError() * errorDifferenceScalar;
	} 
	
	public double getRightAdjustment() {
		return rightEncoderPID.getError() * errorDifferenceScalar;
	}
	
	public void resetEncoders() {
		leftDistanceTraveled += getLeftDistance();
		// leftDistanceTraveled += getTalonPosition(leftMaster);
		rightDistanceTraveled += getRightDistance();
		// rightDistanceTraveled += getTalonPosition(rightMaster);
		// this.resetTalonSRXPositions();
		leftEncoder.reset();
		rightEncoder.reset();
	}

	// 2017 AUTON
	private static void voidMethod(double d) {
	}
	
	public void driveRaw(double left, double right) {
		driveRawLimit(left, right, -1, 1);
	}

	public void driveRawLimit(double left, double right, double maximumNegative, double maximumPositive) {
		driveRawLimit(left, right, maximumNegative, 0, maximumPositive, 0);
	}

	private double prevLeft = 0;
	private double prevRight = 0;

	public void driveRawLimit(double left, double right, double maximumNegative, double minimumNegative,
			double maximumPositive, double minimumPositive) {
		// HAVE TO NEGATE MOTORS
		left = limit(-left, maximumNegative, minimumNegative, maximumPositive, minimumPositive);
		right = limit(-right, maximumNegative, minimumNegative, maximumPositive, minimumPositive);
		prevLeft = left;
		prevRight = right;
		left *= left < 0 ? LEFT_PERCENTAGE_BACKWARD : LEFT_PERCENTAGE_FORWARD;
		right *= right < 0 ? RIGHT_PERCENTAGE_BACKWARD : RIGHT_PERCENTAGE_FORWARD;
		leftMaster.set(ControlMode.PercentOutput, left);
		rightMaster.set(ControlMode.PercentOutput, right);
	}

	private double limit(double value, double maximumNegative, double minimumNegative, double maximumPositive,
			double minimumPositive) {
		if (value < 0) {
			if (value < maximumNegative)
				return maximumNegative;
			else if (value > minimumNegative)
				return minimumNegative;
			else
				return value;
		} else {
			if (value > maximumPositive)
				return maximumPositive;
			else if (value < minimumPositive)
				return minimumPositive;
			else
				return value;
		}
	}

	public double getLeftPIDOutput() {
		return leftEncoderPID.get();
	}

	/**
	 * Gets the right Encoder PID output
	 * 
	 * @return the right Encoder PID output (constrained by the min and max set
	 *         in code)
	 */
	public double getRightPIDOutput() {
		return rightEncoderPID.get();
	}

	private double leftDistanceTraveled = 0;
	private double rightDistanceTraveled = 0;

	public double getLeftRightAdjustment() {
		return (leftEncoderPID.getError() - rightEncoderPID.getError()) * errorDifferenceScalar;
	}

	public double getLeftDistance() {
		return leftEncoder.getDistance();
		// return getTalonPosition(leftMaster);
	}

	/**
	 * Gets the right encoder's distance
	 * 
	 * @return distance in inches measured by the right encoder
	 */
	public double getRightDistance() {
		return rightEncoder.getDistance();
		// return getTalonPosition(rightMaster);
	}

	public boolean leftEncoderOnTarget() {
		return leftEncoderPID.onTarget();
	}

	/**
	 * Checks if the right encoder is within the deadband of the setpoint
	 * 
	 * @return if the right encoder is within the deadband of the setpoint
	 */
	public boolean rightEncoderOnTarget() {
		return leftEncoderPID.onTarget();
	}

	/**
	 * Checks if the left AND right encoders are within the deadband of the
	 * setpoint
	 * 
	 * @return if the left AND right encoders are within the deadband of the
	 *         setpoint
	 */
	public boolean encodersOnTarget() {
		return leftEncoderOnTarget() && rightEncoderOnTarget();
	}

	public void setEncoderPIDS(double p, double i, double d) {
		leftEncoderPID.setPID(p, i, d);
		rightEncoderPID.setPID(p, i, d);
		// System.out.println("Set Encoder PIDs: " + p + ", " + i + ", " + d);
	}

	public void resetEncoderPID() {
		setEncoderPIDS(encoderP, encoderI, encoderD);
	}
	
	public void setLeft(double per) {
		System.out.println("Left Set to "+per);
		leftMaster.set(ControlMode.PercentOutput, per);
	}
	
	public void setRight(double per) {
		System.out.println("Right Set to "+per);
		rightMaster.set(ControlMode.PercentOutput, per);
	}
}
