package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveInCircle extends Command {

	private double stop = 0;
	private double circleRadius = 0;
	private boolean clockwise = true;
	private double rightSpeed = 1.0;
	private double leftSpeed;

	public DriveInCircle(double radius, boolean clockwise) {
		requires(Robot.driveTrain);
		this.circleRadius = radius;
		this.clockwise = clockwise;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoders();
	}
	
	/*
	 * INNER ROTATION POINTS (RIGHT SIDE CW) CREATE AN INNER CIRCLE. FIND CIRCUMFERENCE1
	 * OUTER ROTATION POINTS CREATE AN OUTER CIRCLE. FIND CIRCUMFERENCE2
	 * TAKE [LEFT]SPEED (1) TO INCH RATIO ON CIRCUMFERENCE2 (S2I2)
	 * SPEED TO INCH RATIO ON CIRCUMFERENCE1 (S2I1) SHOULD BE EQUAL TO S2I1. SOLVE FOR SPEED.
	 * SET SPEEDS ACCORDINGLY
	 * EX: 1 speed will carry a radius of outside. x speed will carry a radius of inside proportionately
	 * This allows you to use encoders
	 */

	// Called repeatedly when this Command is scheduled to run
	
	//Gets left and right side radius from center of circle to be rotated
	double innerCircleRadius = circleRadius - Robot.driveTrain.getRobotRadius();
	double outerCircleRadius = circleRadius + Robot.driveTrain.getRobotRadius();
	double placeHolder;
		
	protected void execute() {
		/*If circle radius is 0, don't move.
		 * This line of code may not be necessary, as the radius declaration takes care of
		 * turning on a dime
		if(circleRadius == 0){
			stop++;
		}
		*/
		
		//Finds circumferences of circles to be rotated
		double innerCircumference = Math.PI * 2 * innerCircleRadius;
		double outerCircumference = Math.PI * 2 * outerCircleRadius;
		
		//Speed over distance right is set equal to speed over distance left to find left speed
		// ***right/outer = x/inner ==> inner * right/outer = x***
		double speedRatio = rightSpeed / outerCircumference;
		leftSpeed = speedRatio * innerCircumference;
		
		//Switches directions if going opposite direction
		if(!clockwise){
			//Swaps speeds
			placeHolder = rightSpeed;
			rightSpeed = leftSpeed;
			leftSpeed = placeHolder;
			//Swaps Circle Radii
			placeHolder = innerCircleRadius;
			innerCircleRadius = outerCircleRadius;
			outerCircleRadius = placeHolder;
		}
		
		//If clockwise, rotate clockwise at speeds
		if(clockwise){
			Robot.driveTrain.driveIndependant(-rightSpeed, -leftSpeed);
		}
		//Otherwise, rotate the other way at speeds
		else{
			Robot.driveTrain.driveIndependant(rightSpeed, leftSpeed);
		}
		
		/*
		 * The code inside of here allows the robot to stop after having completed a full rotation
		 * Otherwise, the robot will rotate without stopping.
		 * This could be incorporated with angles 
		//Gets encoder values
		double leftInches = Math.abs(Robot.driveTrain.getLeftEncoder());
		double rightInches = Math.abs(Robot.driveTrain.getRightEncoder());
		
		if (clockwise) {
			if (leftInches >= innerCircumference - 0.2 || rightInches >= outerCircumference - 0.2) {
				Robot.driveTrain.stop();
				stop++;
			}
		}
		else if (!clockwise){
			if (leftInches >= outerCircumference + 0.2 || rightInches >= innerCircumference + 0.2) {
				Robot.driveTrain.stop();
				stop++;
			}
		}
		else{
			Robot.driveTrain.stop();
			stop++;
		}
		*/
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return stop > 5;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
