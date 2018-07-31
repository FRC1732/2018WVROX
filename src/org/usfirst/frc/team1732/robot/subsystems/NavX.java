package org.usfirst.frc.team1732.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class NavX{

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private AHRS navx;
	double heading = 0;

	public NavX(AHRS navx) {
		this.navx = navx;
		configureValues();
	}

	/*
	 * More NavX Values can be added BELOW
	 * First, add a double
	 * Then, in configureValues(), assign that double to navx.get[Parameter]();
	 */

	private void configureValues() {
		heading = navx.getAngle();
	}

	public double getHeading() {
		heading = navx.getAngle() % 360;
		System.out.println(navx.getAngle());
		System.out.println(navx.getAngleAdjustment());
		System.out.println(heading);
		if(heading < 0) {
			heading = 360 + heading;
		}
		return heading;
	}
}
