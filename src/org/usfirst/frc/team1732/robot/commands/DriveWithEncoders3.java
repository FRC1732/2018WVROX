package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithEncoders3 extends Command {

	private double inches = 0;
	private double stop = 0;
	private double speed = 1;
	
	private final double MARGIN = Robot.driveTrain.getMultiplier() + 6.5;

	public DriveWithEncoders3(double inches) {
		requires(Robot.driveTrain);
		this.inches = inches;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoders();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double leftInches = Robot.driveTrain.getLeftEncoder();
		double rightInches = Robot.driveTrain.getRightEncoder();
		
		if (inches < 0){
			Robot.driveTrain.drive(-speed);
		}
		else{
			Robot.driveTrain.drive(speed);
		}
				
		if (inches > 0) {
			if (leftInches >= inches - MARGIN || rightInches >= inches - MARGIN) {
				Robot.driveTrain.stop();
				stop++;
			}
		}
		else if (inches < 0){
			if (leftInches <= inches + MARGIN || rightInches <= inches + MARGIN) {
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
