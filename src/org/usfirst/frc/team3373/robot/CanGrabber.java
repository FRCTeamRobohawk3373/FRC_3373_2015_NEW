package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.Talon;

public class CanGrabber {
	private Talon leftMotor;
	private Talon rightMotor;
	
	public CanGrabber(int leftMotorChannel, int rightMotorChannel){
		leftMotor = new Talon(leftMotorChannel);
		rightMotor = new Talon(rightMotorChannel);
	}
	
	public void controlCanGrabber(int direction){//pass in pov from d-pad
		switch (direction){
			case 0://lower
				lower();
				break;
			case 180://raise
				raise();
				break;
			default:
				stop();
				break;
		}
	}
	public void lowerCanGrabber(){
		lower();
		try{
			Thread.sleep(750);
		} catch(Exception e){
			//Wait
		}
		stop();
		
	}
	public void raiseCanGrabber(){
		raise();
		try{
			Thread.sleep(750);
		} catch(Exception e){
			//Wait
		}
		stop();
	}
	public void lower(){
		leftMotor.set(-0.3);
		rightMotor.set(0.3);
	}
	public void raise(){
		leftMotor.set(0.3);
		rightMotor.set(-0.3);
	}
	public void stop(){
		leftMotor.set(0);
		rightMotor.set(0);
	}
	
}
