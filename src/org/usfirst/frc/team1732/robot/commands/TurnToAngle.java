package org.usfirst.frc.team1732.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1732.robot.Robot;

/**
 *
 */
public class TurnToAngle extends Command {
	
	private int stop = 0;
	private double angle = 0;
	private final double range = 4;
	
	private boolean master = true;
	
	public double getAngle() {
		return angle;
	}

	public void setAngle(double angle) {
		this.angle = angle;
	}

	public TurnToAngle(double angle) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
		this.angle = angle;
	}
	
	public TurnToAngle(double angle, boolean master) {
		// Use requires() here to declare subsystem dependencies
		if(master) {
			requires(Robot.driveTrain);
		}
		this.angle = angle;
		this.master = master;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		//GET HEADING IS ONLY CALLED ONCE BECAUSE SUBSYSTEMS ARE NOT RUN CONTINUOUSLY
		double heading = Robot.navx.getHeading();
		
		SmartDashboard.putNumber("Heading", heading);
		System.out.println("Running");
		System.out.println(heading);
		
		double speed = 0.5;
		
		if(Math.abs(heading - angle) < range * 1.5) {
			speed = (Math.abs(heading - angle) / (range * 1.5)) / 2;
		}
		
		//if the difference is > 180, turn the other way
		if(Math.abs(heading - angle) > 180) {
			speed = -speed;
		}
		
		if (heading < (angle - range / 2.0)) {
			Robot.driveTrain.rotateCW(speed);
			stop = 0;
		} else if (heading > (angle + range / 2.0)) {
			Robot.driveTrain.rotateCCW(speed);
			stop = 0;
		} else {
			Robot.driveTrain.stop();
			stop++;
		}
		
	}
	
	public void runSync() {
		execute();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return stop > 5;
	}
	public boolean isDone() {
		return stop > 5;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
