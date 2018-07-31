package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends Subsystem {
    public static final String NAME = "Flywheel";

    public Flywheel() {
    	super(NAME);
		configureEncoders();
    }
    
    private final double REVS_PER_PULSE = 1 / 1;
	
    //Begin Declaration
	//Motor
	public final static TalonSRX motor = new TalonSRX(RobotMap.FLYWHEEL_MOTOR_DEVICE_NUMBER);
	
	//Encoder
    public TalonEncoder flywheelEncoder;

    @Override
    public void initDefaultCommand() {
    	
    }
    
    public void periodic() {
		SmartDashboard.putNumber("Shooter Encoder Distance: ", flywheelEncoder.getPosition());
	}
    
    private void configureEncoders() {
		flywheelEncoder = new TalonEncoder(motor, FeedbackDevice.QuadEncoder);
		flywheelEncoder.setPhase(true);
		flywheelEncoder.setDistancePerPulse(REVS_PER_PULSE);
		flywheelEncoder.zero();
	}
}
