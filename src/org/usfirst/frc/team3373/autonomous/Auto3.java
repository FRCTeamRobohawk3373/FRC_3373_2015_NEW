package org.usfirst.frc.team3373.autonomous;

import org.usfirst.frc.team3373.robot.SwerveControl;
import org.usfirst.frc.team3373.robot.Indexer;
import org.usfirst.frc.team3373.robot.CanGrabber;
public class Auto3 {
	
	SwerveControl swerve;
	Indexer indexer;
	CanGrabber canGrabber;
	/**
	 * Moves robot into auto zone
	 * @param swerve passed in object of the swerve class
	 * @param indexer passed in object of indexer class
	 * @param canGrabber passed in object of canGrabber
	 */
	public Auto3(SwerveControl swerve, Indexer indexer, CanGrabber canGrabber){
		this.canGrabber = canGrabber;
		this.swerve = swerve;
		this.indexer = indexer;
	}
	
	/**
	 * Indexes can infront of robot
	 * Drops hoopy thing to grab second can
	 * Strafes left into the auto zone
	 */
	public void moveAuto3(){
		//THese values need to be experimentally determined
		//--------
		int target = 3;
		long sleepTime = 1000;
		long moveTime = 1000;
		//--------
		//Close Indexer
		indexer.wheelControl(-1,-1);
		try{
			Thread.sleep(sleepTime);
		}catch(Exception e){
			//do nothing
		}
		indexer.wheelControl(0, 0);
		//Drop hoopy thing	
		swerve.relativeMoveRobot(180, .5, moveTime);
	}
}

