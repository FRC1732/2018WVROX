package org.usfirst.frc.team1732.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//Joysticks
	public static final int LEFT_JOYSTICK_Y_AXIS = 1;
    public static final int RIGHT_JOYSTICK_Y_AXIS = 1;
    
    public static final int LEFT_JOYSTICK_X_AXIS = 0;
    public static final int RIGHT_JOYSTICK_X_AXIS = 0;

    public static final int LEFT_JOYSTICK_USB = 0;
    public static final int RIGHT_JOYSTICK_USB = 1;
    
    //Solenoid
    public static final int PCM_CAN_ID = 0;
    
    //Drivetrain
    public static final int LEFT_MASTER_MOTOR_DEVICE_NUMBER = 14;
    public static final int LEFT_1_MOTOR_DEVICE_NUMBER = 13;
    public static final int LEFT_2_MOTOR_DEVICE_NUMBER = 11;

    public static final int RIGHT_MASTER_MOTOR_DEVICE_NUMBER = 21; //19
    public static final int RIGHT_1_MOTOR_DEVICE_NUMBER = 22; //20
    public static final int RIGHT_2_MOTOR_DEVICE_NUMBER = 23; //21
    
    //Encoders
    public static final int LEFT_ENCODER_CHANNEL_A = 2;
    public static final int LEFT_ENCODER_CHANNEL_B = 3;
    public static final int RIGHT_ENCODER_CHANNEL_A = 0;
    public static final int RIGHT_ENCODER_CHANNEL_B = 1;
    
    //Shooter
    public static final int FLYWHEEL_MOTOR_DEVICE_NUMBER = 4;
    
    //Grabber
    public static final int GRABBER_SOLENOID_DEVICE_NUMBER = 0;
    
    
    //Roller
    public static final int ROLLER_SOLENOID_DEVICE_NUMBER = 3;
    public static final int ROLLER_MOTOR_DEVICE_NUMBER = 12;
    
    
}
