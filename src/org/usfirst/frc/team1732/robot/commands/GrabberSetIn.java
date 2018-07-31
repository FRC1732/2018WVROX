package org.usfirst.frc.team1732.robot.commands;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class GrabberSetIn extends InstantCommand {

    public GrabberSetIn() {
    	super();
    	requires(Robot.grabber);
    }
    
    @Override
    protected void initialize() {
    	//System.out.println("THE GRABBERSETIN HAS BEEN INITIALIZED");
    	Robot.grabber.grab();
    }

}
