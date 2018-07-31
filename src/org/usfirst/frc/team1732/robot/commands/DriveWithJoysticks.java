package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.driveTrain;
import static org.usfirst.frc.team1732.robot.Robot.oi;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
//DRIVINGROBOT PROJECT
public class DriveWithJoysticks extends Command {

	public DriveWithJoysticks() {
		super("Drive With Joysticks");
		requires(driveTrain);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//System.out.println("drive with joysticks is SUPER EXECUTED CURRENTLY");
		driveTrain.driveWithJoysticks(oi.getLeftSpeed(), oi.getRightSpeed());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}
}
