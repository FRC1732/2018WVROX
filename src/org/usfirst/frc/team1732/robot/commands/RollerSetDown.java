package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class RollerSetDown extends InstantCommand {

    public RollerSetDown() {
        super();
    	requires(Robot.roller);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.roller.setDown();
    }

}
