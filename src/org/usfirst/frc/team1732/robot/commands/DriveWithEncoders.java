package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.driveTrain;

import org.usfirst.frc.team1732.robot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveWithEncoders extends Command {
	
	public static double P = 1.8;
	public static double I = 0.1;
	public static double D = 0.0;
	
	public static double DIVISOR = 1.0;
	
	//public static double INCHES = 36;

	private static PIDController leftDistance = new PIDController(P, I, D, Robot.driveTrain.leftEncoder, d->{});
	private static PIDController rightDistance = new PIDController(P, I, D, Robot.driveTrain.rightEncoder,d->{});
	
	static {
		LiveWindow.add(leftDistance);
    	LiveWindow.add(rightDistance);
	}
	
	private static double inches; //Setpoint
	private final double TOLERANCE = 0.0;
	
		
    public DriveWithEncoders(double inches) {
        requires(Robot.driveTrain);
		DriveWithEncoders.inches = inches;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.resetEncoders();
    	
    	leftDistance.setSetpoint(inches);
    	leftDistance.setAbsoluteTolerance(TOLERANCE);
    	leftDistance.enable();
    	
    	rightDistance.setSetpoint(inches);
    	rightDistance.setAbsoluteTolerance(TOLERANCE);
    	rightDistance.enable();
    	
    	//Robot.driveTrain.resetEncoderPIDValues();
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Left OUTPUT: " + leftDistance.get());
    	System.out.println("Right OUTPUT: " + rightDistance.get());
//    	System.out.println("Left Encoder Position" + driveTrain.getLeftDistance());
//		System.out.println("Right Encoder Position" + driveTrain.getRightDistance());
    	driveTrain.driveIndependant(leftDistance.get(), rightDistance.get());
    	
//		SmartDashboard.putNumber("Left Encoder", driveTrain.getLeftDistance());
//		SmartDashboard.putNumber("Right Encoder", driveTrain.getRightDistance());

    	
    	/*
		driveTrain.setLeftEncoderSetpoint(inches);
		driveTrain.setRightEncoderSetpoint(inches);
		double leftOutput = driveTrain.getLeftAdjustment();
		double rightOutput = driveTrain.getRightAdjustment();
		
		System.out.println("Right Output: " + rightOutput);
		System.out.println("Left Output: " + leftOutput);
		
		if(leftOutput > 1){
			leftOutput = 1;
		}
		if(leftOutput < -1){
			leftOutput = -1;
		}
		
		if(rightOutput > 1){
			rightOutput = 1;
		}
		if(rightOutput < -1){
			rightOutput = -1;
		}
		
		System.out.println("Right Output: " + rightOutput);
		System.out.println("Left Output: " + leftOutput);
		
		driveTrain.driveIndependant(leftOutput, rightOutput);
		*/
    	
//    	if(leftDistance.onTarget()) {
//    		leftDistance.disable();
//    	}
//    	if(rightDistance.onTarget()) {
//    		rightDistance.disable();
//    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return (!leftDistance.isEnabled() || leftDistance.onTarget()) &&
        		//(!rightDistance.isEnabled() || rightDistance.onTarget());
    	
    	return false;
        
        /*
         * 				>.<
         */
    }

    // Called once after isFinished returns true
    protected void end() {
    	leftDistance.disable();
    	rightDistance.disable();
    	Robot.driveTrain.drive(0);
    }
}
