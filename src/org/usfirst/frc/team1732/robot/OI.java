package org.usfirst.frc.team1732.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc.team1732.robot.commands.ExampleCommand;
import org.usfirst.frc.team1732.robot.commands.GrabberSetIn;
import org.usfirst.frc.team1732.robot.commands.GrabberSetOut;
import org.usfirst.frc.team1732.robot.commands.RollerSetDown;
import org.usfirst.frc.team1732.robot.commands.RollerSetUp;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    private Joystick left = new Joystick(RobotMap.LEFT_JOYSTICK_USB);
    private Joystick right = new Joystick(RobotMap.RIGHT_JOYSTICK_USB);
    
	private final Button grabber = new JoystickButton(right, 2);
	private final Button roller = new JoystickButton(right, 3);

	public OI(){
		//Grabber
		grabber.whenPressed(new GrabberSetIn());
		grabber.whenReleased(new GrabberSetOut());
		
		//Roller
		roller.whenPressed(new RollerSetDown());
		roller.whenReleased(new RollerSetUp());
	}    //Nate was here
    
    public double getLeftSpeed() {
    	//return -controller.getRawAxis(1);// for use with game controller
    	return -left.getRawAxis(RobotMap.LEFT_JOYSTICK_Y_AXIS);
    	// return -buttons.getRawAxis(1);
    }

    public double getRightSpeed() {
    	//return -controller.getRawAxis(3);// for use with game controller
    	return -right.getRawAxis(RobotMap.RIGHT_JOYSTICK_Y_AXIS);
    	// return -buttons.getRawAxis(3);
    }
    
    public double getLeftTurn() {
    	return left.getRawAxis(RobotMap.LEFT_JOYSTICK_X_AXIS);
    }
    
    public double getRightTurn() {
    	return right.getRawAxis(RobotMap.RIGHT_JOYSTICK_X_AXIS);
    }
}
