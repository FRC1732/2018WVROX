package org.usfirst.frc.team1732.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DrivingAroundLikeHentaiLord extends CommandGroup {

	public DrivingAroundLikeHentaiLord() {
		System.out.println("This is happening, but you're just wrong, kiddo.");
		//addSequential(new DriveWithEncoders(24));
//		addSequential(new DriveWithEncoders(-12));
		/*
		addSequential(new Pause(1000));
		addSequential(new TurnToAngleWithEncoders(90));
		//addSequential(new TurnToAngle(90));
		addSequential(new Pause(750));
		addSequential(new DriveWithEncoders(12));
		addSequential(new Pause(1000));
		addSequential(new TurnToAngleWithEncoders(90));
		//addSequential(new TurnToAngle(180));
		addSequential(new Pause(750));
		addSequential(new DriveWithEncoders(12));
		addSequential(new Pause(1000));
		addSequential(new TurnToAngleWithEncoders(90));
		//addSequential(new TurnToAngle(270));
		addSequential(new Pause(750));
		addSequential(new DriveWithEncoders(12));
		addSequential(new Pause(1000));
		addSequential(new TurnToAngleWithEncoders(90));
		//addSequential(new TurnToAngle(0));
		addSequential(new Pause(750));
		*/
		
		addSequential(new TurnToAngleWithEncoders(90));
		addSequential(new Pause(750));
		addSequential(new TurnToAngleWithEncoders(-180));
		addSequential(new Pause(750));
		addSequential(new TurnToAngleWithEncoders(90));

		

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
