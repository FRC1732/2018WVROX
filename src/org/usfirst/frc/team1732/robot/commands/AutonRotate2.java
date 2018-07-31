package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonRotate2 extends CommandGroup {

	public AutonRotate2() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.
		Robot.ahrs.zeroYaw();

		addSequential(new TurnToAngle(50));
		addSequential(new Pause(5000));
		addSequential(new TurnToAngle(-30));
		addSequential(new Pause(5000));
		addSequential(new TurnToAngle(270));
		addSequential(new Pause(5000));
		addSequential(new TurnToAngle(-10));

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
	}
}
