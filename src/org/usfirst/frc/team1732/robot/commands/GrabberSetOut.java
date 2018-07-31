package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class GrabberSetOut extends InstantCommand {

    public GrabberSetOut() {
    	super();
    	requires(Robot.grabber);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void initialize() {
    	Robot.grabber.release();
    }
   
}
