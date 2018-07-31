package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.driveTrain;

import org.usfirst.frc.team1732.robot.Robot;
import org.usfirst.frc.team1732.robot.monitoring.PositionMonitoring;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class PointTurns extends Command {

	private PositionMonitoring p;
	
    public PointTurns(PositionMonitoring p) {
    	super("Point Turning");
		requires(driveTrain);
		this.p = p;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double angle = p.getHeading() % 360;
    	if(angle < 0) {
    		angle = 360 + angle;
    	}
    	SmartDashboard.putNumber("Heading", angle);
    	if(angle < 80) {
    		Robot.driveTrain.rotateCW(0.5);
    	}else if(angle > 120) {
    		Robot.driveTrain.rotateCCW(0.5);
    	}else {
    		Robot.driveTrain.stop();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }
    
}
