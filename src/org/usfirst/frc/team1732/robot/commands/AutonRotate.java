package org.usfirst.frc.team1732.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.commands.TurnToAngle;

/**
 *
 */
public class AutonRotate extends Command {

	
	public AutonRotate() {
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		new TurnToAngle(50).start();
		new TurnToAngle(180).start();
		new TurnToAngle(20).start();
		new TurnToAngle(-100).start();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
