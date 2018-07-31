
package org.usfirst.frc.team1732.robot;

import org.usfirst.frc.team1732.robot.commands.AutonRotate;
import org.usfirst.frc.team1732.robot.commands.AutonRotate2;
import org.usfirst.frc.team1732.robot.commands.AutonRotate3;
import org.usfirst.frc.team1732.robot.commands.DriveInCircle;
import org.usfirst.frc.team1732.robot.commands.DrivingAroundLikeHentaiLord;
import org.usfirst.frc.team1732.robot.commands.PointTurns;
import org.usfirst.frc.team1732.robot.commands.Pause;
import org.usfirst.frc.team1732.robot.commands.TurnToAngle;
import org.usfirst.frc.team1732.robot.conf.Config;
import org.usfirst.frc.team1732.robot.monitoring.PositionMonitoring;
import org.usfirst.frc.team1732.robot.subsystems.DriveTrain;
import org.usfirst.frc.team1732.robot.subsystems.ExampleSubsystem;
import org.usfirst.frc.team1732.robot.subsystems.Flywheel;
import org.usfirst.frc.team1732.robot.subsystems.Grabber;
import org.usfirst.frc.team1732.robot.subsystems.NavX;
import org.usfirst.frc.team1732.robot.subsystems.Roller;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	public static Config config;
	
	public static OI oi;
	public static DriveTrain driveTrain;
	public static Grabber grabber;
	public static Roller roller;
	public static AHRS ahrs;
	public static PositionMonitoring pm;
	public static NavX navx;
	public static Flywheel flywheel;

	// Compressor c = new Compressor(0);

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		config = Config.load();
		
		System.out.println("YO, THE ROBOT IS STARTING THOUGH");
		try {
			System.out.println("Robot turning on");
			// c.setClosedLoopControl(true);
			//ahrs = new AHRS(Port.kMXP);
			ahrs = new AHRS(SPI.Port.kMXP);
			
			System.out.println(ahrs.isConnected());
			System.out.println("THE THING DIDNOT FAIL!!1!");
			
			initializeSubsystems();
			// System.out.println(driveTrain.leftEncoder);
			oi = new OI();
			navx = new NavX(ahrs);
			//pm = new PositionMonitoring(ahrs);

			new Thread(pm).start();
		} catch (Exception e) {
			e.printStackTrace();
			System.out.println(e);
			System.out.println(e.getMessage());
		}
	}

	private long time;

	public void robotPeriodic() {
		SmartDashboard.putNumber("Execution Time", System.currentTimeMillis() - time);
		time = System.currentTimeMillis();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		/*
		 * PUT AUTONOMOUS COMMANDS BELOW HERE:
		 */

		//autonomousCommand = new PointTurns(pm);
		//autonomousCommand = new AutonRotate2();
		autonomousCommand = new DrivingAroundLikeHentaiLord();
		//autonomousCommand = new AutonRotate3();
		//autonomousCommand = new DriveInCircle(50, true);

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null){
			autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		ahrs.zeroYaw();
		System.out.println("teleopInit Called");
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		Scheduler.getInstance().removeAll(); // Cancels commands
		
		// DriveWithJoysticks cb = new DriveWithJoysticks();
		// cb.start();		
		Robot.driveTrain.resetEncoders();
    	Robot.driveTrain.resetEncoderPIDValues();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		// System.out.println("YO, THE ROBOT IS TELEOP-ING THOUGH");
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		//LiveWindow.run();
	}

	public void initializeSubsystems() {
		driveTrain = new DriveTrain();
		grabber = new Grabber();
		roller = new Roller();
		flywheel = new Flywheel();
		System.out.println("Subsystems Initialized");
	}
}
