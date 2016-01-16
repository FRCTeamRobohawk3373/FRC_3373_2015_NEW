package org.usfirst.frc.team3373.autonomous;

import org.usfirst.frc.team3373.robot.Indexer;
import org.usfirst.frc.team3373.robot.Lifter;
import org.usfirst.frc.team3373.robot.SwerveControl;

public class Auto2 {
	
	SwerveControl swerve;
	Indexer indexer;
	Lifter lifter;
	
	public Auto2(SwerveControl swerve, Indexer indexer, Lifter lifter){
		this.swerve = swerve;
		this.indexer = indexer;
		this.lifter = lifter;

	}

	public void runAuto2(){
		goToCanRimHeight();
		
		while (lifter.getLeftPotValue() >= lifter.heightToPosition(27)){
			lifter.goToLength();
		}
		
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		liftCanPastToteHeight();
		
		while (lifter.getLeftPotValue() >= lifter.heightToPosition(40)){
			lifter.goToLength();
		}
		swerve.relativeMoveRobot(90, .5, 1);
		while (indexer.isHolding()){
			pickUpTote();
		}
		swerve.relativeMoveRobot(180, .5, 5);
	}
	
	private void pickUpTote(){
		indexer.controlArms(1);
		indexer.wheelControl(.5, .5);
	}
	
	private void goToCanRimHeight(){
		lifter.changeTarget(lifter.heightToPosition(27));
	}
	
	private void liftCanPastToteHeight(){
		lifter.changeTarget(lifter.heightToPosition(40));
	}
}
