package org.usfirst.frc.team3373.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lifter {
	CANTalon leftActuator;//currently ID 5
	CANTalon rightActuator;//currently ID 6

	LookupTable lookupTable = new LookupTable();
	
	double pr = 10;
	double ir = 10;
	double dr = 2;
	
	double pl = 10;
	double il = 0;
	double dl = 2;
	
	//height in inches
	double robotHeight = 60;
	
	//parallelogram lift length in inches
	double armLength = 60;
	
	double cosTheta;
	
	//difference in position in pot value
	double offset = .05;
	
	//distance between lift joint and actuator joint on height
	double jointThetaH = 12;
	
	//distance between lift joint and actuator joint on arm
	double jointThetaA = 24;
	
	double potScalarR = 783;
	double potScalarL = 763;
	double casingLength = 12;
	//Right Actuator THIS CALIBRATION IS CURRENTLY BROKEN
	double maxPotValueR = 964.0;
	double minPotValueR = 134.0;
	double maxLengthR = 12.3125;//in inches
	double minLengthR = 1.6875;//in inches
	//left Actuator THIS CALIBRATION IS CURRENTLY BROKEN
	double maxPotValueL = 957.0;
	double minPotValueL = 139.0;
	double maxLengthL = 12.25;//in inches
	double minLengthL = 1.6875;//in inches
	
	double diffBetweenPots = 0;
	
	double lifterTarget;
	
	boolean isRunning = false;
	
	boolean hasAlreadyMoved = false;
	
	double speedConstant = .1;

	
	//Have to use these as a work around to the PID controller class
	PIDOutputObject leftActPIDOutput = new PIDOutputObject();
	PIDOutputObject rightActPIDOutput = new PIDOutputObject();
	PIDOutputObject errorPIDOutput = new PIDOutputObject();
	
	PIDInputObject leftActPIDInput = new PIDInputObject();
	PIDInputObject rightActPIDInput = new PIDInputObject();
	PIDInputObject errorPIDInput = new PIDInputObject();
	
	PIDController leftActPID = new PIDController(10, 0, 5, leftActPIDInput, leftActPIDOutput);
	PIDController rightActPID = new PIDController(10, 0, 5, rightActPIDInput, rightActPIDOutput);
	PIDController errorPID = new PIDController(3, 0, 0, errorPIDInput, errorPIDOutput);
	
	//New Variables
	double targetHeight = 5;
	double targetRightPot;
	double targetLeftPot;
	double deltaPotR;
	double deltaPotL;
	double modifierR = 1 * .90;
	double modifierL = 1;
	double rightSpeed;
	double leftSpeed;
	double maxLifterSpeed = 60;
	double minLifterSpeed = 20;
	double maxLifterSpeedR = maxLifterSpeed;//In pot units per 10 milliseconds
	double minLifterSpeedR = minLifterSpeed;
	double maxLifterSpeedL = maxLifterSpeed;
	double minLifterSpeedL = minLifterSpeed;
	
	double inchesOffGroundR;
	double inchesOffGroundL;
	double deltaInchesBetweenActuators;
	double errorCompensation; //this will be applied to maxLifterSpeed L
	double errorCompensationConstant = .3;
	double deltaInchesL;
	double deltaInchesR;
	
	int shutdownCounter;
	
	//Calibration relating height of hook off ground to pot values on each arm
	
	double[] heightOffGround = {2,3,4,5,6,7,8,9,10,11};//In inches

	double[] heightOffGroundL = {0, 4.875, 9, 14.875, 21.8125, 30.6875, 39.875, 46.063, 49.8125, 55.625};
	double[] leftPot = {141, 225, 294, 385, 487, 610, 731, 806, 856, 926};
	double[] heightOffGroundR = {0, 4.1875, 9.375, 14.875, 19.125, 26.125, 33.75, 41.5, 50.0625, 55.6875};
	double[] rightPot = {179, 248, 331, 417, 479, 578, 680, 780, 886, 953};
	
//	double[] heightOffGroundL = {1.375, 6.125, 11.75, 20, 28, 35.375, 43.5, 48.5, 53.625, 55.125};
//	double[] leftPot = {168, 247, 336, 459, 573, 672, 777, 840, 903, 920};
//	double[] heightOffGroundR = {0, 2, 5.25, 12.375, 21.375, 29.875, 42.625, 49.125, 53.125, 54.5};
//	double[] rightPot = {178, 211, 264, 377, 509, 627, 794, 874, 922, 939};
	
	
	/**
	 * Initializes Lifter class, feeds in motor values to control lifting motors
	 * @param leftActuatorID CANBus ID for left actuator
	 * @param rightActuatorID CANBus ID for right actuator
	 */
	public Lifter(int leftActuatorID, int rightActuatorID){
		leftActuator = new CANTalon(leftActuatorID);
		rightActuator = new CANTalon(rightActuatorID);
		
		
		/*
		//Sets PID
		rightActuator.setPID(pr, ir, dr);
		leftActuator.setPID(pl, il, dl);
		
		//Sets value to be fed to CANTalons (Position in this case, v -1 to 1 speed)
		rightActuator.changeControlMode(CANTalon.ControlMode.Position);
		leftActuator.changeControlMode(CANTalon.ControlMode.Follower);
		
		//Sets main type of sensor available on the CANBus
		 * */
		 
		rightActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		leftActuator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
		
		leftActuator.changeControlMode(CANTalon.ControlMode.Speed);
		rightActuator.changeControlMode(CANTalon.ControlMode.Speed);
		
		//leftActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		//rightActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		
		lifterTarget = getLeftActuatorLength();
		
		initPIDControllers();
	}
	
	/**
	 * Given a target position, goes to the position
	 * @param targetPos position from 0-5 volts the lifter needs to go to
	 */
	/*
	public void goToPosition(double targetPos){
			rightActuator.set(heightToPosition(targetPos));
			leftActuator.set(rightActuator.getDeviceID());
	}*/
	
	/**
	 * Converts target height to a position on the potentiometer 
	 * @param targetHeight target lifter height above the ground
	 * @return target pot position (0-5V)
	 */
	
	public double heightToPosition(double targetHeight){
		double actuatorLength;
		cosTheta = (robotHeight - targetHeight)/armLength;
		actuatorLength = Math.sqrt(Math.pow(jointThetaH, 2) + Math.pow(jointThetaA, 2) - (2*jointThetaH*jointThetaA*cosTheta));
		
		return actuatorLengthToPot(actuatorLength, potScalarR);
	}
	
	/**
	 * given a target actuator length, returns a pot position
	 * @param actuatorTargetLength target length for to hit a certain lifter height
	 * @return encoder position (0-5)
	 */
	
	private double actuatorLengthToPot(double actuatorTargetLength, double potScalar){
		double potPosition;
		potPosition = (actuatorTargetLength-casingLength) * (potScalar/casingLength);
		
		return potPosition;
	}
	
	private double potValueToActuatorLength(double potValue, double potScalar){
	
		double actLength;
		actLength = (potValue) * (casingLength/potScalar);
	
		return actLength;
	}
	
	/*
	 * Jamie's Code
	 */
	
	
	public double getLeftPotValue(){
		return leftActuator.getAnalogInPosition();

	}
	
	public double getRightPotValue(){
		return leftActuator.getAnalogInPosition() + diffBetweenPots;
	}
	
	public void initPIDControllers(){
		leftActPID.enable();
		rightActPID.enable();
		errorPID.enable();
		//set input range
		leftActPID.setInputRange(minLengthL, maxLengthL);
		rightActPID.setInputRange(minLengthR, maxLengthR);
		//set output range
		leftActPID.setOutputRange(-0.2, 0.2);
		rightActPID.setOutputRange(-0.2, 0.2);
		errorPID.setInputRange(0, 1);
		errorPID.setOutputRange(-0.1, 0.1);
		errorPID.setSetpoint(0);
		
		leftActPID.setPercentTolerance(5);
		rightActPID.setPercentTolerance(5);
		errorPID.setPercentTolerance(100);
		//update PIDs
		updatePIDControllers();
	}
	
	public void updatePIDControllers(){
		leftActPIDInput.setValue(getLeftActuatorLength());
		//SmartDashboard.putNumber("Current Length", leftActuator.getAnalogInPosition());
		//SmartDashboard.putNumber("CurrentPos", leftActuator.getAnalogInPosition());
		rightActPIDInput.setValue(getRightActuatorLength());
		errorPIDInput.setValue(getLeftActuatorLength() - getRightActuatorLength());
	}
	
	public double getLeftActuatorLength(){
		double slope  = (maxLengthL - minLengthL)/(maxPotValueL - minPotValueL);
		double potValue = leftActuator.getAnalogInRaw();
		double length;
		length = slope * (potValue - maxPotValueL) + maxLengthL;
		return length;
	}
	public double getRightActuatorLength(){
		double slope = (maxLengthR - minLengthR)/(maxPotValueR - minPotValueR);
		SmartDashboard.putNumber("Slope", slope);
		double potValue = rightActuator.getAnalogInRaw();
		SmartDashboard.putNumber("RightActuatorPot", potValue);
		double length;
		length = (slope * (potValue - maxPotValueR)) + maxLengthR;
		return length;
	}
	
	public int inchesToPotR(double lengthInInches){
		int potValue = (int)(lengthInInches * (maxPotValueR/maxLengthR));
		return potValue;
	}
	
	public int inchesToPotL(double lengthInInches){
		int potValue = (int)(lengthInInches * (maxPotValueL/maxLengthL));
		return potValue;
	}
	
	/*public void extendLeft(double target){
		if(Math.abs(target-leftActuator.getAnalogInPosition()) > 2){
			updatePIDControllers();
			leftActPID.setSetpoint(target);//inchesToPotValueL(target));
			SmartDashboard.putNumber("Target: ", target);//inchesToPotValueL(target));
			SmartDashboard.putNumber("Speed of Left Talon", leftActPIDOutput.getPIDValue());
			leftActuator.set(leftActPIDOutput.getPIDValue());
		}
	}*/
	
	/**
	 * Sets the target acuator length
	 * @param target target actuator length
	 */
	public void changeTarget(double target){
		if (target > maxLengthL){
			lifterTarget = maxLengthL;
		}  else if (target < minLengthL){
			lifterTarget = minLengthL;
		} else {
			lifterTarget = target;
		}
	}
	
	
	public void goToLength(){//in inches
		//if((Math.abs(lifterTarget - getLeftActuatorLength()) > .15) || (Math.abs(lifterTarget - getRightActuatorLength()) > .15)){
			updatePIDControllers();
			//set setpoint for Pid loops
			leftActPID.setSetpoint(lifterTarget);
			rightActPID.setSetpoint(lifterTarget);
		
			SmartDashboard.putNumber("Target: ", lifterTarget);
			
		
			SmartDashboard.putNumber("Left Current Length: ", getLeftActuatorLength());
			SmartDashboard.putNumber("Right Current Length: ", getRightActuatorLength());
		
			SmartDashboard.putNumber("PID Output Left: ", leftActPIDOutput.getPIDValue());
			SmartDashboard.putNumber("PID Output Right: ", rightActPIDOutput.getPIDValue());
			SmartDashboard.putNumber("PID Output Error: ", errorPIDOutput.getPIDValue());
		
			//set motors to desired speed
			System.out.println("Moving MotorL: " + leftActPIDOutput.getPIDValue());
			System.out.println("PotValueL" + leftActuator.getAnalogInPosition());
			
			System.out.println("Moving MotorR: " + rightActPIDOutput.getPIDValue());
			System.out.println("PotValuer" + rightActuator.getAnalogInPosition());
			
			leftActuator.set(leftActPIDOutput.getPIDValue());// + errorPIDOutput.getPIDValue());
			SmartDashboard.putNumber("Left Actuator Speed: ", leftActuator.get());
			rightActuator.set(rightActPIDOutput.getPIDValue());// - errorPIDOutput.getPIDValue());
			SmartDashboard.putNumber("Right Actuator Speed: ", rightActuator.get());//.getPIDValue() + errorPIDOutput.getPIDValue());
		//}
			//Does not allow the robot to break itself by throwing arm out of wack
			/*
			if ((leftActuator.isFwdLimitSwitchClosed() || rightActuator.isFwdLimitSwitchClosed()) && ((lifterTarget > getLeftActuatorLength()) || (lifterTarget > getRightActuatorLength()))){
				leftActuator.set(0);
				rightActuator.set(0);
			} else if ((leftActuator.isRevLimitSwitchClosed() || rightActuator.isRevLimitSwitchClosed()) && ((lifterTarget < getLeftActuatorLength()) || (lifterTarget < getRightActuatorLength()))){
				leftActuator.set(0);
				rightActuator.set(0);
			} else {

			}*/

	}
	/**
	 * Raises the lifterArm for manual control
	 */
	public void raise(){
		changeTarget(lifterTarget += .01);
	}
	/**
	 * Lowers the lifter arm for manual control
	 */
	public void lower(){
		changeTarget(lifterTarget -= .01);
	}
	
	public void absoluteRaise(){
		leftActuator.set(.4);
		rightActuator.set(.4);
	}
	
	public void absoluteLower(){
		leftActuator.set(-.4);
		rightActuator.set(-.4);
	}
	
	public void absoluteStop(){
		leftActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		rightActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		leftActuator.set(0);
		rightActuator.set(0);
	}
	
	public void moveLeft(int direction){
		leftActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		leftActuator.set(.4 * direction);
	}
	
	public void moveRight(int direction){
		rightActuator.changeControlMode(CANTalon.ControlMode.PercentVbus);
		rightActuator.set(.4 * direction);
	}
	
	public void printPotValues(){
		
		SmartDashboard.putNumber("LeftPotRaw", leftActuator.getAnalogInRaw());
		SmartDashboard.putNumber("RightPotRaw", rightActuator.getAnalogInRaw());
		
		SmartDashboard.putBoolean("LeftFWD", leftActuator.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("LeftREV", leftActuator.isRevLimitSwitchClosed());
		SmartDashboard.putBoolean("RightFWD", rightActuator.isFwdLimitSwitchClosed());
		SmartDashboard.putBoolean("RightREV", rightActuator.isRevLimitSwitchClosed());
		
		SmartDashboard.putNumber("Target", lifterTarget);
		
		SmartDashboard.putNumber("LeftHeight", getLeftActuatorLength());
		SmartDashboard.putNumber("RightHeight", getRightActuatorLength());
	}
	/**
	 * Always add to rightActuatorSpeed
	 * @return modifier to cause right actuator to catch up to left
	 */
	public double speedModifier(){
		double speedModifier;
		speedModifier = (getLeftActuatorLength() - getRightActuatorLength()) * speedConstant;
		return speedModifier;
	}
	
	public void threadedGoToPos(boolean isEnabled){
		Thread thread = new Thread(new Runnable(){
			public void run(){
				isRunning = true;
				System.out.println("In thread");
				/*while (Math.abs(targetPosition - getLeftActuatorLength()) > .1 ){
					if ((targetPosition > getLeftActuatorLength())){// && (Math.abs(getLeftActuatorLength() - getRightActuatorLength()) < .15)){
						leftActuator.set(.5);
						System.out.println("Going up");
					} else if ((targetPosition < getLeftActuatorLength())){// && (Math.abs(getLeftActuatorLength() - getRightActuatorLength()) < .15)){
						leftActuator.set(-.5);
						System.out.println ("Going down");
					} else if (targetPosition == getLeftActuatorLength()){
						leftActuator.set(0);
					}
					
					if (targetPosition > getRightActuatorLength()){
						rightActuator.set(.5);

					} else if (targetPosition < getRightActuatorLength()){
						rightActuator.set(-.5);
					} else if (targetPosition == getRightActuatorLength()){
						leftActuator.set(0);
					}
					System.out.println("In thread");
				
				}*/
				if (getRightActuatorLength() < lifterTarget){//getLeftActuatorLength() < lifterTarget || getRightActuatorLength() < lifterTarget) {
						double speed = 0.3;
						double leftSpeed = 0.0;
						double rightSpeed = 0.0;
						while ((lifterTarget - getRightActuatorLength()) > 0){//(lifterTarget - getLeftActuatorLength()) > 0 || (lifterTarget - getRightActuatorLength()) > 0){
							if ((lifterTarget - getLeftActuatorLength()) > 0){
								leftSpeed = speed;
							} else {
								leftSpeed = 0;
							}
							
							if ((lifterTarget - getRightActuatorLength()) > 0){
								rightSpeed = speed;
							} else {
								rightSpeed = 0;
							}
							double offset = getLeftActuatorLength() - getRightActuatorLength();
							
							SmartDashboard.putNumber("Offset", offset);

							double slope = .1;
							//rightSpeed += offset * slope;
							
							leftActuator.set(leftSpeed);
							rightActuator.set(rightSpeed);
							hasAlreadyMoved = true;
						}
				}
			
				
				if ((getRightActuatorLength() > lifterTarget) && !hasAlreadyMoved){//(getLeftActuatorLength() > lifterTarget || getRightActuatorLength() > lifterTarget) && !hasAlreadyMoved) {
					double speed = -0.3;
					double leftSpeed = 0.0;
					double rightSpeed = 0.0;
					while ((lifterTarget - getRightActuatorLength()) < 0){//(lifterTarget - getLeftActuatorLength()) < 0 || (lifterTarget - getRightActuatorLength()) < 0){
						if ((lifterTarget - getLeftActuatorLength()) < 0){
							leftSpeed = speed;
						} else {
							leftSpeed = 0;
						}
						
						if ((lifterTarget - getRightActuatorLength()) < 0){
							rightSpeed = speed;
						} else {
							rightSpeed = 0;
						}
						double offset = getLeftActuatorLength() - getRightActuatorLength();
						
						SmartDashboard.putNumber("Offset", offset);

						double slope = 1;
						//rightSpeed += offset * slope;
						
						leftActuator.set(leftSpeed);
						rightActuator.set(rightSpeed);
					}
			}
				/*if (!hasAlreadyMoved && ((getLeftActuatorLength() > lifterTarget || getRightActuatorLength() > lifterTarget))){
					while ((getLeftActuatorLength() != lifterTarget || getRightActuatorLength() != lifterTarget)){
						if (getLeftActuatorLength() > lifterTarget){
							if (speedModifier() < 0){
								leftActuator.set(-.3 - speedModifier());
							} else {
								leftActuator.set(-.3);
							}
							System.out.println("Ldown");
						} else {
							leftActuator.set(0);
						}
						
						if (getRightActuatorLength() > lifterTarget){
							if (speedModifier() > 0){
								rightActuator.set(-.3 + speedModifier());
							} else {
								rightActuator.set(-.3);
							}
							System.out.println ("Rdown");
						} else {
							rightActuator.set(0);
						}
						
						System.out.println(hasAlreadyMoved);
					}
				}*/
				
				
				leftActuator.set(0);
				rightActuator.set(0);
				hasAlreadyMoved = false;
				isRunning = false;
			}
		});
		
		if (!isRunning){
			thread.start();
		}
	}
	
	public void changeTargetHeight(double heightOfHookOffGroundInInches){
		targetHeight = heightOfHookOffGroundInInches;
		//Checking boundaries
		if(targetHeight > heightOffGroundL[heightOffGroundL.length - 1]){
			targetHeight = heightOffGroundL[heightOffGroundL.length - 1];
		} else if(targetHeight < heightOffGroundL[0]){
			targetHeight = heightOffGroundL[0];
		}
	}
	
	public void relativeChangeTargetHeight(double heightDeltaInInches){
		//double newHeight;
		//newHeight = lookupTable.lookUpValue(leftActuator.getAnalogInRaw(), leftPot, heightOffGroundL);
		//newHeight += heightDeltaInInches;
		//targetHeight = newHeight;
		
		targetHeight += heightDeltaInInches;
		//Checking boundaries
		if(targetHeight > heightOffGroundL[heightOffGroundL.length - 1]){
			targetHeight = heightOffGroundL[heightOffGroundL.length - 1];
		} else if(targetHeight < heightOffGroundL[0]){
			targetHeight = heightOffGroundL[0];
		}
	}
	
	public void goToHeightOffGround(){
		targetRightPot = lookupTable.lookUpValue(targetHeight, heightOffGroundR, rightPot);
		targetLeftPot = lookupTable.lookUpValue(targetHeight, heightOffGroundL, leftPot);
		
		inchesOffGroundL = lookupTable.lookUpValue(leftActuator.getAnalogInRaw(), leftPot, heightOffGroundL);
		inchesOffGroundR = lookupTable.lookUpValue(rightActuator.getAnalogInRaw(), rightPot, heightOffGroundR);

//		if (Robot.SmartON) {
			SmartDashboard.putNumber("Target Right Pot: ", targetRightPot);
			SmartDashboard.putNumber("Target Left Pot: ", targetLeftPot);
			SmartDashboard.putNumber("Target Height inches: ", targetHeight);
			SmartDashboard.putNumber("Inches Above Ground L:", inchesOffGroundL);
			SmartDashboard.putNumber("Inches Above Ground R:", inchesOffGroundR);
			SmartDashboard.putNumber(" Left Pot:", leftActuator.getAnalogInRaw());
			SmartDashboard.putNumber("Right Pot:", rightActuator.getAnalogInRaw());
//		}
		
		//deltaPotR = targetRightPot - rightActuator.getAnalogInRaw();
		//deltaPotL = targetLeftPot - leftActuator.getAnalogInRaw();
		
		deltaInchesBetweenActuators = inchesOffGroundL - inchesOffGroundR;
		// need to compensate if actuators are not together
		errorCompensation=deltaInchesBetweenActuators*0.5; // TODO: need to adjust scale to optimize compensation speed
		if(errorCompensation>0.5) errorCompensation=0.5;   // limit compensation factor
		if(errorCompensation<-0.5) errorCompensation=-0.5; // limit compensation factor
		
		deltaInchesR = targetHeight-inchesOffGroundR;
		if(Math.abs(deltaInchesR)>0.2) {
			// need to move right actuator
			rightSpeed=Math.abs((deltaInchesR+2)*(deltaInchesR+2)*(deltaInchesR+2));  
			if (deltaInchesR<-0.2) rightSpeed= -1*rightSpeed; // need to move right actuator down
			// limit speed  TODO: need to find good speed limits
			if(rightSpeed>0 && rightSpeed<minLifterSpeed) rightSpeed=minLifterSpeed;    // min speed up
			if(rightSpeed>maxLifterSpeed) rightSpeed=maxLifterSpeed; 					// max speed up
			if(rightSpeed<0 && rightSpeed>-minLifterSpeed) rightSpeed=-minLifterSpeed;  // min speed down
			if(rightSpeed<-maxLifterSpeed) rightSpeed=-maxLifterSpeed;   				// max speed down
		}
		else rightSpeed=0; // stop right actuator
		rightActuator.set(rightSpeed);
			//TODO: /2
		deltaInchesL = targetHeight-inchesOffGroundL;				
		if (Math.abs(deltaInchesL)>0.2) {
			// need to move left actuator
			leftSpeed=Math.abs((deltaInchesL+2)*(deltaInchesL+2)*(deltaInchesL+2));  // scales speed vs distance to go
			if(deltaInchesL<-0.2) leftSpeed= -1*leftSpeed;  // need to move left actuator down
			leftSpeed=leftSpeed; //*(1+errorCompensation);  TODO: compensation is currently disabled; need to put back in
			// limit speed TODO: need to find good speed limits
			if(leftSpeed>0 && leftSpeed<minLifterSpeed) leftSpeed=minLifterSpeed;   // min speed up
			if(leftSpeed>maxLifterSpeed) leftSpeed=maxLifterSpeed; 					 // max speed up
			if(leftSpeed<0 && leftSpeed>-minLifterSpeed) leftSpeed=-minLifterSpeed; // min speed down
			if(leftSpeed<-maxLifterSpeed) leftSpeed=-maxLifterSpeed; 				 // max speed down
		}
		else leftSpeed=0; // stop left actuator
		leftActuator.set(leftSpeed);
		
//		if (Robot.SmartON) {
			SmartDashboard.putNumber("Inches Difference between Actuators:", deltaInchesBetweenActuators);
			SmartDashboard.putNumber("Error Compensation Value:", errorCompensation);
			SmartDashboard.putNumber("Delta Inches Between Actuators:", deltaInchesBetweenActuators);
			SmartDashboard.putNumber("Left Lifter Speed:", leftSpeed);
			SmartDashboard.putNumber("Right Lifter Speed:", rightSpeed);
//		}
	}
		
	
	public void ORGgoToHeightOffGround(){
		if(shutdownCounter < 5){
		
			targetRightPot = lookupTable.lookUpValue(targetHeight, heightOffGroundR, rightPot);
			targetLeftPot = lookupTable.lookUpValue(targetHeight, heightOffGroundL, leftPot);
		
		
			inchesOffGroundL = lookupTable.lookUpValue(leftActuator.getAnalogInRaw(), leftPot, heightOffGroundL);
			inchesOffGroundR = lookupTable.lookUpValue(rightActuator.getAnalogInRaw(), rightPot, heightOffGroundR);
			
			
			if(leftActuator.getAnalogInRaw() < targetLeftPot){
				deltaInchesBetweenActuators = inchesOffGroundL - inchesOffGroundR;
			} else {
				deltaInchesBetweenActuators = inchesOffGroundR - inchesOffGroundL;
			}
		
			if(Math.abs(deltaInchesBetweenActuators) >= 0.5){//TODO: Make this 1/2 at competition
				shutdownCounter +=1;
			} else {
				if(shutdownCounter > 0){
					shutdownCounter -=1;
				}
			}

			SmartDashboard.putNumber("Difference between Actuators (in):", deltaInchesBetweenActuators);
			SmartDashboard.putNumber("Counter for shutdown mode", shutdownCounter);
		
			SmartDashboard.putNumber("Target Height: ", targetHeight);
			SmartDashboard.putNumber("Target Right Pot: ", targetRightPot);
			SmartDashboard.putNumber("Target Left Pot: ", targetLeftPot);
		
			deltaPotR = targetRightPot - rightActuator.getAnalogInRaw();
			deltaPotL = targetLeftPot - leftActuator.getAnalogInRaw();
		
			
			if(deltaInchesBetweenActuators > 0.53){
				errorCompensation = 0.2;
			} else if(deltaInchesBetweenActuators < -0.53){
				errorCompensation = 1.8;
			} else{
				errorCompensation = 1 - (deltaInchesBetweenActuators * errorCompensationConstant);
			}
			
			rightSpeed = modifierR * deltaPotR;
			leftSpeed = modifierL * deltaPotL;
		
			maxLifterSpeedL = maxLifterSpeed * errorCompensation;
			
			if(rightSpeed > maxLifterSpeedR){
				rightSpeed = maxLifterSpeedR;
			} else if(rightSpeed < -maxLifterSpeedR){
				rightSpeed = -maxLifterSpeedR;
			} else if(rightSpeed > 0 && rightSpeed < minLifterSpeedR){
				rightSpeed = minLifterSpeedR;
			} else if(rightSpeed < 0 && rightSpeed > -minLifterSpeedR){
				rightSpeed = -minLifterSpeedR;
			}
		
			if(leftSpeed > maxLifterSpeedL){
				leftSpeed = maxLifterSpeedL;
			} else if(leftSpeed < -maxLifterSpeedL){
				leftSpeed = -maxLifterSpeedL;
			} else if(leftSpeed > 0 && leftSpeed < minLifterSpeedL){
				leftSpeed = minLifterSpeedL;
			} else if(leftSpeed < 0 && leftSpeed > -minLifterSpeedL){
				leftSpeed = -minLifterSpeedL;
			}
		
			SmartDashboard.putNumber("Right Speed: ", rightSpeed);
			SmartDashboard.putNumber("Left Speed", leftSpeed);
			
			if(Math.abs(targetRightPot - rightActuator.getAnalogInRaw()) > 4){
				rightActuator.set(rightSpeed);
			} else if(shutdownCounter > 50){
				rightActuator.set(0);
			} else {
				rightActuator.set(0);
			}
			
			if(Math.abs(targetLeftPot - leftActuator.getAnalogInRaw()) > 4){
				leftActuator.set(leftSpeed);
			} else if(shutdownCounter > 50){
				leftActuator.set(0);
			} else{
				leftActuator.set(0);
			}
			/*
			deltaPotR = targetPosition - getRightActuatorLength();
			deltaPotL = targetPosition - getLeftActuatorLength();
			
			rightSpeed = modifierR * inchesToPotR(deltaPotR);
			leftSpeed = modifierL * inchesToPotL(deltaPotL);
			
	
			SmartDashboard.putNumber("TargetPos: ", targetPosition);
			SmartDashboard.putNumber("deltaPotR", deltaPotR);
			SmartDashboard.putNumber("deltaPotL", deltaPotL);
			
			if(rightSpeed > maxLifterSpeed){
				rightSpeed = maxLifterSpeed;
			} else if(rightSpeed < -maxLifterSpeed){
				rightSpeed = -maxLifterSpeed;
			} else if(rightSpeed > 0 && rightSpeed < minLifterSpeed){
				rightSpeed = minLifterSpeed;
			} else if(rightSpeed < 0 && rightSpeed > -minLifterSpeed){
				rightSpeed = -minLifterSpeed;
			}
			
			if(leftSpeed > maxLifterSpeed){
				leftSpeed = maxLifterSpeed;
			} else if(leftSpeed < -maxLifterSpeed){
				leftSpeed = -maxLifterSpeed;
			} else if(leftSpeed > 0 && leftSpeed < minLifterSpeed){
				leftSpeed = minLifterSpeed;
			} else if(leftSpeed < 0 && leftSpeed > -minLifterSpeed){
				leftSpeed = -minLifterSpeed;
			}
			
			SmartDashboard.putNumber("Right Speed: ", rightSpeed);
			SmartDashboard.putNumber("Left Speed", leftSpeed);
			
			
			if(Math.abs(targetPosition - getRightActuatorLength()) > 0.1){
				rightActuator.set(rightSpeed);
			} else {
				rightActuator.set(0);
			}
			if(Math.abs(targetPosition - getLeftActuatorLength()) > 0.1){
				leftActuator.set(leftSpeed);
			} else {
				leftActuator.set(0);
			}
			*/
		} else {
			// Error counter is Above 50,
			// stop lifter
			leftActuator.set(0);
			rightActuator.set(0);
		}
	}
	
	
	
	public void manualUp(double trigger){
		if(trigger < 0.2) trigger = 0;  // ignore trigger if barely pulled
		

		inchesOffGroundL = lookupTable.lookUpValue(leftActuator.getAnalogInRaw(), leftPot, heightOffGroundL);
		inchesOffGroundR = lookupTable.lookUpValue(rightActuator.getAnalogInRaw(), rightPot, heightOffGroundR);
		
		// Check for over travel
		if((shutdownCounter > 5) || (inchesOffGroundL >= heightOffGroundL[heightOffGroundL.length - 1]) || (inchesOffGroundR >= heightOffGroundR[heightOffGroundR.length - 1])){
			// Stop actuators
			leftActuator.set(0);
			rightActuator.set(0);
			return;
		} else {
			
			deltaInchesBetweenActuators = inchesOffGroundL - inchesOffGroundR;
			
			if(deltaInchesBetweenActuators > 0.53){
				//Left above right >> Slow left
				errorCompensation = 0.2;
			} else if(deltaInchesBetweenActuators < -0.53){
				//Right above left >> Speed left
				errorCompensation = 1.8;
			} else{
				errorCompensation = 1 - (deltaInchesBetweenActuators * errorCompensationConstant);
			}
			
			leftSpeed = errorCompensation * trigger * maxLifterSpeed;
			rightSpeed = trigger * maxLifterSpeed;
			
			leftActuator.set(leftSpeed);
			rightActuator.set(rightSpeed);
		}
		
	}
	
	
	public void manualDown(double trigger){
		if(trigger < 0.2) trigger = 0; // ignore trigger if barely pulled

		// going down is negative
		trigger = -trigger;

		inchesOffGroundL = lookupTable.lookUpValue(leftActuator.getAnalogInRaw(), leftPot, heightOffGroundL);
		inchesOffGroundR = lookupTable.lookUpValue(rightActuator.getAnalogInRaw(), rightPot, heightOffGroundR);
		
		if(shutdownCounter > 5|| (inchesOffGroundL <= heightOffGroundL[0]) || (inchesOffGroundR <= heightOffGroundR[0])){
			leftActuator.set(0);
			rightActuator.set(0);
			return;
		} else {
			
			deltaInchesBetweenActuators = inchesOffGroundR - inchesOffGroundL;
			
			if(deltaInchesBetweenActuators > 0.53){
				errorCompensation = 0.2;
			} else if(deltaInchesBetweenActuators < -0.53){
				errorCompensation = 1.8;
			} else{
				errorCompensation = 1 - (deltaInchesBetweenActuators * errorCompensationConstant);
			}
			
			leftSpeed = errorCompensation * trigger * maxLifterSpeed;
			rightSpeed = trigger * maxLifterSpeed;
			
			leftActuator.set(leftSpeed);
			rightActuator.set(rightSpeed);
		}
	}
	
	
	public void manualStop(){
		leftActuator.set(0);
		rightActuator.set(0);
	}
	
	
	/**
	 * Get current height of hook off the ground
	 * @return value returned is in inches
	 */
	public double getCurrentHeight(){
		double height;
		height = lookupTable.lookUpValue(leftActuator.getAnalogInRaw(), leftPot, heightOffGroundL);
		return height;
	}
	
}
