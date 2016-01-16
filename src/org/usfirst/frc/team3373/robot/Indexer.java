package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer {
	
	Timer armTimer = new Timer();
	
	private RobotDrive indexer;
	private CANTalon armMotor;
	private double max;
	private double min;
	private double maxOutput = 0.4;//TODO CALIBRATE
	private double output = maxOutput;
	private double current;
	private double maxCurrent = 1.0; //TODO CALIBRATE
	private double minCurrent = 0.15;//TODO CALIBRATE
	
	public boolean isHookArmCollisionPossible;
	private boolean isFirstIteration = true;
	private double timeToAvoidCollision = 2;//TODO CALIBRATE
	
	/**
	 * Initializes an indexer object. 
	 * @param leftWheelChannel The channel where the left wheel is plugged in. 
	 * @param rightWheelChannel The channel where the right wheel is plugged in.
	 * @param armMotorChannel
	 * @param potAnalogChannel
	 */

	public Indexer(int leftWheelChannel , int rightWheelChannel, int armMotorID){
		indexer = new RobotDrive(leftWheelChannel, rightWheelChannel);
		
		armMotor = new CANTalon(armMotorID);
		
		armMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
		armMotor.enableBrakeMode(true);
		armMotor.enableLimitSwitch(false, false);
		
		//armTimer.start(); used for collision prevention, that didn't work
	}
	
	public void wheelControl(double leftY, double rightY){
		if(leftY > 0.1){
			leftY = 1;
		} else if(leftY < -0.1){
			leftY = -1;
		} else{
			leftY = 0;
		}
		if(rightY > 0.1){
			rightY = 1;
		} else if(rightY < -0.1){
			rightY = -1;
		} else{
			rightY = 0;
		}
		indexer.tankDrive(leftY, rightY);
	}
	/**
	 * Controls the opening and closing of arms on the robot
	 * @param LX
	 */
	public void controlArms(double LX){
		
		current = armMotor.getOutputCurrent();
		if(current > maxCurrent){
			output = 0.1;
		} else if(current < minCurrent){
			output += 0.05;
		}
		
		if(output > maxOutput){
			output = maxOutput;
		} else if(output < 0){
			output = 0;
		}
		SmartDashboard.putNumber("Current: ", armMotor.getOutputCurrent());
		SmartDashboard.putNumber("Output: ", output);
		SmartDashboard.putNumber("Left Axis: ", LX);
		if(LX > 0.1){
			armMotor.set(-output);
			isHookArmCollisionPossible = true;
		} else if(LX < -0.1) {
			armMotor.set(output);
		} else{
			armMotor.set(0);
		}
	}
	
	//Test code TODO Take this method out
	/*
	public void controlArms(double x){
		if(x>0){
			indexer.tankDrive(1, 1);
		} else{
			indexer.tankDrive(0, 0);
		}
	}*/
	
	

	
	public boolean isHolding(){
		if(output < (maxOutput)){
			return true;
		} else {
			return false;
		}
	}
	/*This was an attempt avoid collisions between arms and lifter, doesn't work currently
	public void avoidCollision(boolean collisionPossible){
		double startTime = 0;
		double currentTime;
		if(collisionPossible){
			currentTime = armTimer.get();
			if(isFirstIteration == true){
				startTime = armTimer.get();
				isFirstIteration = false;
				System.out.println("First Iteration");
			}
			if(isFirstIteration == false && ((currentTime-startTime) < timeToAvoidCollision)){
				controlArms(1);
				System.out.println("Opening");
			} else{
				controlArms(0);
				isHookArmCollisionPossible = false;
				isFirstIteration = true;
				System.out.println("Stop");
			}
		} else {
			//do nothing
		}
	}*/

	/*
	public void controlMotors(double LX, double RX){
		
		leftCurrent = leftArmMotor.getOutputCurrent();
		rightCurrent = rightArmMotor.getOutputCurrent();
		
		//check current on left
		if(leftCurrent > maxCurrent){
			leftOutput -= 0.05;
		} else if(leftCurrent < minCurrent){
			leftOutput += 0.05;
		}
		if(leftOutput > maxOutput){
			leftOutput = maxOutput;
		} else if(leftOutput < 0){
			leftOutput = 0;
		}
		
		//check current on right
		if(rightCurrent > maxCurrent){
			rightOutput -= 0.05;
		} else if(rightCurrent < minCurrent){
			rightOutput += 0.05;
		}
		if(rightOutput > maxOutput){
			rightOutput = maxOutput;
		} else if(rightOutput < 0){
			rightOutput = 0;
		}
		
		SmartDashboard.putNumber("Left Current: ", leftCurrent);
		SmartDashboard.putNumber("Right Current: ", rightCurrent);

		SmartDashboard.putNumber("Left Output ", leftOutput);
		SmartDashboard.putNumber("Right Output ", rightOutput);
		
		SmartDashboard.putNumber("Left Axis: ", LX);
		SmartDashboard.putNumber("Right Axis: ", RX);
		
		if(LX > 0.1){
			leftArmMotor.set(leftOutput);
		} else if(LX < -0.1){
			leftArmMotor.set(-leftOutput);
		} else{
			leftArmMotor.set(0);
		}
		
		if(RX > 0.1){
			rightArmMotor.set(rightOutput);
		} else if(RX < -0.1){
			rightArmMotor.set(-rightOutput);
		} else{
			rightArmMotor.set(0);
		}
		
		
		if(LX > 0.1){
			leftArmMotor.set(leftOutput);
		} else if(LX < 0.1){
			leftArmMotor.set(-leftOutput);
		} else {
			leftArmMotor.set(0);
		}
		
		if(RX > 0.1){
			rightArmMotor.set(rightOutput);
		} else if(RX < 0.1){
			rightArmMotor.set(-rightOutput);
		} else {
			rightArmMotor.set(0);
		}
	}*/

}