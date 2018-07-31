package org.usfirst.frc.team1732.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveInches extends CommandGroup {

    public DriveInches(double forward, double backwards) {
	    addSequential(new DriveWithEncoders2(forward, backwards));
    }
}
