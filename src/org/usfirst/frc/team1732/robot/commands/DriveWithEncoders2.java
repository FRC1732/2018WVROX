package org.usfirst.frc.team1732.robot.commands;

import static org.usfirst.frc.team1732.robot.Robot.driveTrain;

import org.usfirst.frc.team1732.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveWithEncoders2 extends Command {

	private final double leftDistance;
	private final double rightDistance;

	public DriveWithEncoders2(double distanceInches) {
		this(distanceInches, distanceInches);
	}

	public DriveWithEncoders2(double leftInches, double rightInches) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(driveTrain);
		leftDistance = leftInches;
		rightDistance = rightInches;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		driveTrain.resetEncoders();
		driveTrain.setLeftEncoderSetpoint(leftDistance);
		driveTrain.setRightEncoderSetpoint(rightDistance);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println("Left Distance: " + leftDistance);
		System.out.println("Right Distance: " + rightDistance);
		System.out.println("LeftPIDError: " + driveTrain.getLeftPIDError());
		System.out.println("RightPIDError: " + driveTrain.getRightPIDError());
		
		if (Math.abs(driveTrain.getLeftPIDError()) < DriveTrain.ENCODER_IZONE
				|| Math.abs(driveTrain.getRightPIDError()) < DriveTrain.ENCODER_IZONE) {
			driveTrain.setEncoderPIDS(DriveTrain.encoderP, DriveTrain.ENCODER_IZONE_I, DriveTrain.encoderD);
		} else {
			driveTrain.resetEncoderPIDValues();
		}

		double leftOutput = driveTrain.getLeftPIDOutput();
		double rightOutput = driveTrain.getRightPIDOutput();
		
		System.out.println("LeftRaw: " + driveTrain.getLeftPIDOutput());
		System.out.println("RightRaw: " + driveTrain.getRightPIDOutput());

		if (leftDistance == rightDistance) {
			double adjust = driveTrain.getLeftRightAdjustment();
			// System.out.println("Left-right adjustment: " + adjust);
			// if (driveTrain.getLeftPIDError() < 0) {
			// leftOutput = leftOutput - adjust;
			// } else {
			leftOutput = leftOutput + adjust;
			// }
			// if (driveTrain.getRightPIDError() < 0) {
			// rightOutput = rightOutput + adjust;
			// } else {
			rightOutput = rightOutput - adjust;
			// }
			System.out.println("Adjust: " + adjust);
		}

		double max = Math.max(Math.abs(leftOutput), Math.abs(rightOutput));
		System.out.println("Max: " + max);

		if (max > 1) {
			leftOutput = leftOutput / max;
			rightOutput = rightOutput / max;
		}
		
		System.out.println("I want to drive at: " + leftOutput + " and " + rightOutput + ", but I'm just not able to!");
		driveTrain.driveRaw(leftOutput, rightOutput);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return driveTrain.encodersOnTarget();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		driveTrain.driveRaw(0, 0);
		driveTrain.resetEncoderPIDValues();
	}

}
