package org.usfirst.frc.team1732.robot.subsystems;

import org.usfirst.frc.team1732.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Grabber extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public static final String NAME = "Grabber";
	
	public Grabber(){
		super(NAME);
	}
	
//	private final Solenoid grabberPosition	= new Solenoid(RobotMap.PCM_CAN_ID, RobotMap.GRABBER_SOLENOID_DEVICE_NUMBER);
	
	public static final boolean	OUT		= false;
	public static final boolean	IN	= !OUT;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void grab() {
    	//System.out.println("THE GRAB METHOD HAS BEEN CALLED");
//    	grabberPosition.set(IN);
    }
    
    public void release(){
//    	grabberPosition.set(OUT);
    }
    
}

