package org.usfirst.frc.team3373.autonomous;

import org.usfirst.frc.team3373.robot.Lifter;
import org.usfirst.frc.team3373.robot.SwerveControl;

public class Auto1 {
	
	SwerveControl swerve;
	Lifter lifter;
	/**
	 * Moves robot into auto zone
	 * @param swerve passed in object of the swerve class
	 */
	public Auto1(SwerveControl swerve, Lifter lifter){
		this.swerve = swerve;
		this.lifter = lifter;
	}
	
	/**
	 * Moves the robot forward five seconds at 50% speed
	 */
	public void moveAuto1(){
		//THese values need to be experimentally determined
		//--------
		int target = 3;
		double time = 1;
		//--------
		lifter.changeTarget(target);
		lifter.goToLength();
		swerve.relativeMoveRobot(90, .5, time);
	}
}
