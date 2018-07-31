package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.driveTrain;
import static org.usfirst.frc.team1732.robot.Robot.oi;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
//DRIVINGROBOT PROJECT
public class DriveWithArcade extends Command {

	public DriveWithArcade() {
		super("Drive With Arcade");
		requires(driveTrain);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//System.out.println("drive with joysticks is SUPER EXECUTED CURRENTLY");
		driveTrain.driveWithArcade(oi.getLeftTurn(), oi.getRightSpeed());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}
}
