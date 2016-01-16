package org.usfirst.frc.team3373.autonomous;

import org.usfirst.frc.team3373.robot.SwerveControl;

public class Auto0 {
	
	SwerveControl swerve;
	/**
	 * Moves robot into auto zone
	 * @param swerve passed in object of the swerve class
	 */
	public Auto0(SwerveControl swerve){
		this.swerve = swerve;
	}
	
	/**
	 * Moves the robot forward five seconds at 50% speed
	 */
	public void runAuto0(){
		swerve.relativeMoveRobot(0, .5, 5);
	}
}
