package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.driveTrain;
import static org.usfirst.frc.team1732.robot.Robot.oi;

import org.usfirst.frc.team1732.robot.Robot;

import static org.usfirst.frc.team1732.robot.Robot.navx;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
//DRIVINGROBOT PROJECT
public class DriveWithStick extends Command {

	public DriveWithStick() {
		super("Drive With Stick");
		requires(driveTrain);
		Robot.ahrs.zeroYaw();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//System.out.println("drive with joysticks is SUPER EXECUTED CURRENTLY");
		driveTrain.driveWithStick(oi.getRightSpeed(), oi.getRightTurn(), navx.getHeading());
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}
}
