package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngleWithEncoders extends Command {

	private double angle = 0;
	private double stop = 0;
	private double speed = 0.5;

	public TurnToAngleWithEncoders(double angle) {
		requires(Robot.driveTrain);
		this.angle = angle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoders();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double leftInches = Robot.driveTrain.getLeftEncoder();
		double rightInches = Robot.driveTrain.getRightEncoder();
		double circumference = Robot.driveTrain.getRotateCircumference();
		double rotateDistance = circumference * (angle / 360.0);
		
		//SOMETHING IS WRONG WITH DRIVING BACKWARDS
		if (angle < 0){
			Robot.driveTrain.rotateCCW(speed);
		}
		else if (angle > 0){
			Robot.driveTrain.rotateCW(speed);
		}
		else{
			stop++;
		}
		
		if (rotateDistance > 0) {
			if (leftInches >= rotateDistance - 0.05 || rightInches <= -rotateDistance + 0.05) {
				Robot.driveTrain.stop();
				stop++;
			}
		}
		else if (rotateDistance < 0){
			if (leftInches <= rotateDistance + 0.05 || rightInches >= -rotateDistance - 0.05) {
				Robot.driveTrain.stop();
				stop++;
			}
		}
		else{
			Robot.driveTrain.stop();
			stop++;
		}
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
