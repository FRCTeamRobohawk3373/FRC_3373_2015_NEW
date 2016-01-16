
package org.usfirst.frc.team3373.robot;


import com.kauailabs.nav6.frc.IMU; 
import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
public class Robot extends SampleRobot {
    //Controllers
	SuperJoystick driver;
    SuperJoystick shooter;
    //Robot Subsystems
    Indexer indexer;
    SwerveControl swerve;
    Lifter lifter;
    Timer robotTimer;

    
    //Controllers
    DigitalInput ones;
    DigitalInput twos;
    DigitalInput fours;
    DigitalInput eights;
    
    
    SerialPort serial_port;
    //IMU imu;  // Alternatively, use IMUAdvanced for advanced features
    IMUAdvanced imu;
    
    //Joystick Axes
    int LX = 0;
    int LY = 1;
    int Ltrigger = 2;
    int Rtrigger = 3;
    int RX = 4;
    int RY = 5;
    
    boolean SmartON = true;
    boolean first_iteration;
    int index;//used for 16 switch
    

    /***************************
     * Robot Talon Identifier  *
     *		F                  *
     * 0 ------ 1              *
     * |        |              *
     * |        |              *
     * 2--------3              *
     ***************************/
    int frontLeftRotate = 0;
    int frontRightRotate = 1;
    int backLeftRotate = 2;
    int backRightRotate = 3;
    
    int frontLeftDrive = 0;
    int frontRightDrive = 1;
    int backLeftDrive = 2;
    int backRightDrive = 3;

    boolean haveRun;
        
    double robotWidth = 21.125;//TODO CALIBRATE check
    double robotLength = 33.5;//TODO CALIBRATE check
    double rotateRadius = 0.;
    double objectDistance = 60.;
    double stackDistance = 45.;
    
    boolean ismanualLifterMode = false;
    boolean manualLifting=false;
    boolean isCollisionPossible = false;
    
    
    public Robot() {
    	//Initialize controllers
        driver = new SuperJoystick(0);
        shooter = new SuperJoystick(1);
        //Initialize robot sub-systems
        lifter = new Lifter(5, 4);
        indexer = new Indexer(4, 5, 6);
        swerve = new SwerveControl(frontLeftDrive, frontLeftRotate, frontRightDrive, 
        		frontRightRotate, backLeftDrive, backLeftRotate, backRightDrive, 
        	    backRightRotate, robotWidth, robotLength);
        
        
        //LimitSwitches for Auto selector
        ones = new DigitalInput(6);
        twos = new DigitalInput(7);
        fours = new DigitalInput(8);
        eights = new DigitalInput(9);

        robotTimer = new Timer();
        
        
        haveRun = false;

        
        try {
        	serial_port = new SerialPort(57600,SerialPort.Port.kMXP);
    		
    		// You can add a second parameter to modify the 
    		// update rate (in hz) from 4 to 100.  The default is 100.
    		// If you need to minimize CPU load, you can set it to a
    		// lower value, as shown here, depending upon your needs.
    		
    		// You can also use the IMUAdvanced class for advanced
    		// features.
    		
    		byte update_rate_hz = 50;
    		//imu = new IMU(serial_port,update_rate_hz);
    		imu = new IMUAdvanced(serial_port,update_rate_hz);
        	} catch(Exception ex) {
        		
        	}
        first_iteration = true;
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
    	// TODO CALIBRATE TEST 16 SWITCH
    	int index = 0;//for testing purposes
    	if(ones.get()){
    		index += 1;
    	}
    	if(twos.get()){
    		index += 2;
    	}
    	if(fours.get()){
    		index += 4;
    	}
    	if(eights.get()){
    		index += 8;
    	}
    	System.out.println(index);
    	switch(index){
    		case 0://Simply drive into auto zone
    			indexer.controlArms(-1);
    			try{
    				Thread.sleep(1000);
    			} catch(Exception e){
    				//do nothing
    			}
    			lifter.manualUp(1);
    			try{
    				Thread.sleep(500);
    			} catch(Exception e){
    				//do nothing
    			}
    			lifter.manualStop();
    			swerve.relativeMoveRobot(90, 1, 3);
    			break;
    		case 1://Start behind items and drive forward, hopefully bringing can and tote to the middle
    			indexer.controlArms(-1);
    			try{
    				Thread.sleep(1000);
    			} catch(Exception e){
    				//do nothing
    			}
    			swerve.calculateSwerveControl(1, 0, 0);
    			indexer.wheelControl(-1, -1);
    			try{
    				Thread.sleep(3500);
    			} catch (Exception x){
    				//Do nothing
    			}
    			swerve.calculateSwerveControl(0, 0, 0);
    			indexer.wheelControl(0, 0);
    			break;
    		case 2:
    			indexer.controlArms(-1);
    			try{
    				Thread.sleep(1000);
    			} catch(Exception e){
    				//Do nothing
    			}
    			lifter.changeTargetHeight(27);
    			while(lifter.getCurrentHeight() <= 26.5){
    				lifter.goToHeightOffGround();
    			}
    			lifter.manualStop();
    			swerve.relativeMoveRobot(270, 1, 4);
    			break;
    		case 3:
    			indexer.controlArms(-1);
    			try{
    				Thread.sleep(1000);
    			} catch(Exception e){
    				//do nothing
    			}
    			lifter.manualUp(1);
    			try{
    				Thread.sleep(500);
    			} catch(Exception e){
    				//do nothing
    			}
    			lifter.manualStop();
    			swerve.relativeMoveRobot(90, 0, 1);
    			swerve.relativeMoveRobot(90, 1, 3);
    			break;
    		case 4:
    			break;
    		case 5:
    			break;
    		case 6:
    			break;
    		case 7:
    			break;
    		case 8:
    			break;
    		case 9:
    			break;
    		case 10:
    			break;
    		case 11:
    			break;
    		case 12:
    			break;
    		case 13:
    			break;
    		case 14:
    			break;
    		case 15:
    			//swerve.wheelsToZero();
    			break;
    		default:
    			//swerve.relativeRotateRobot(60);
    	}
    	
    	
    	/*swerve.relativeMoveRobot(270, 0.3, 2);
    	swerve.relativeMoveRobot(0, 0.3, 2);
    	swerve.relativeMoveRobot(90, 0.3, 2);
    	swerve.relativeMoveRobot(180, 0.3, 2);
    	
    	double angle = 0;
    			
    	while(angle < 360){
        	swerve.relativeMoveRobot(angle, 0.3, 0.2);
        	angle += 10;
    	}*/
    	/*
    	try{
    		Thread.sleep(4000);
    		} catch(Exception e){
    		
    	}
    	swerve.relativeRotateRobot(-10);*/
    	
    }

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        
		ismanualLifterMode = false;	// start in NOT manual lifter mode
		manualLifting=false;
		lifter.changeTargetHeight(lifter.getCurrentHeight());  // current lifter height is wherever we are at startup
		
		while (isOperatorControl() && isEnabled()) {
			/****************************
    		 * Driver Specific Controls *
    		 ****************************/
    		//Driving
    		if(driver.isLBHeld()){
    			// LB: Turbo Mode
    			swerve.setSpeedMode(0.5);
    		} else if(driver.isRBHeld()){
    			// RB: Sniper Mode
    			swerve.setSpeedMode(0.20);
    		} else{
    			// Regular speed mode
    			swerve.setSpeedMode(0.3);
    		}
    		
    		/*
            if(driver.isLStickPushed()){
            	swerve.switchToFieldCentric();
            }
            if(driver.getRawAxis(Rtrigger) > 0.2){
            	swerve.switchToObjectCentric();
            }
            if(driver.getRawAxis(Ltrigger) > 0.2){
            	swerve.switchToHookCentric();
            }
            if(driver.isRStickPushed()){
            	swerve.switchToRobotCentric();
            }*/
    		
    		//This is using momentary switches for hook and tote centric
    		if(driver.getRawAxis(Rtrigger) > 0.2){
            	swerve.switchToObjectCentric();
            } else if(driver.getRawAxis(Ltrigger) > 0.2){
            	swerve.switchToHookCentric();
            } else {
            	swerve.switchToRobotCentric();
            }
    		
    		swerve.move(-driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX));
    		
   
    		/* This is nick's pretty code, not yet working
    		//Switching Driving modes
    		if(driver.isRStickHeld()){
    			rotateRadius = 0.0;
    			swerve.isFieldCentric = false;
    		} else if(driver.isLStickPushed()){	
    			swerve.isFieldCentric = true;
    		} else if(driver.getRawAxis(Rtrigger) > 0.2){
    			rotateRadius = objectDistance;
    			swerve.isFieldCentric = false;
    		} else if(driver.getRawAxis(Ltrigger) > 0.2){
    			rotateRadius = stackDistance;
    			swerve.isFieldCentric = false;
    		}
    		swerve.swerveControl(driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX), rotateRadius);
			*/
    		

    		/*****************************
    		 * Shooter Specific Controls *
    		 *****************************/
    		
    		//Index and arm control
    		indexer.wheelControl(shooter.getRawAxis(LY), shooter.getRawAxis(RY));
    		indexer.controlArms(shooter.getRawAxis(LX));  //TODO: need to add control to RX also
    		
    		/* This was an attempt avoid collisions between arms and lifter, doesn't work currently
    		if(shooter.isStartPushed()){//TODO Take this if statement out
    			indexer.isHookArmCollisionPossible = true;
    		}
    		
    		if(indexer.isHookArmCollisionPossible){//AND height is in conflict zone AND hook is moving
    			isCollisionPossible = true;
    		} else{
    			isCollisionPossible = false;
    		}
    		indexer.avoidCollision(isCollisionPossible);
    		*/
    		
    		// Lifter control -- (Shooter left & right buttons)
    		if(shooter.isLBPushed()){  
    			// LB = move lifter down by X inches
    			if(ismanualLifterMode) lifter.changeTargetHeight(lifter.getCurrentHeight());
    			lifter.relativeChangeTargetHeight(-12); // was -12 //TODO CALIBRATE
    			ismanualLifterMode = false;
    		} else if(shooter.isRBPushed()){  
    			// RB = move lifter up by X inches
    			if(ismanualLifterMode) lifter.changeTargetHeight(lifter.getCurrentHeight());
    			lifter.relativeChangeTargetHeight(12);  //was 12 //TODO CALIBRATE
    			ismanualLifterMode = false;
    		}
    		
    		// Old Manual lifter controls -- (Shooter left & right triggers)
    		/*
    		if((shooter.getRawAxis(Rtrigger) > 0.2) && (shooter.getRawAxis(Ltrigger) > 0.2)){
    			// both triggers pulled: go into manual lifter mode but stop lifter
    			ismanualLifterMode = true;
    			lifter.manualStop();
    		} else if(shooter.getRawAxis(Ltrigger) > 0.2){
    			// LT pulled: go into manual lifter mode and so move lifter DOWN 
    			ismanualLifterMode = true;
    			lifter.manualDown(shooter.getRawAxis(Ltrigger));
    		} else if(shooter.getRawAxis(Rtrigger) > 0.2){
    			// RT pulled: go into manual lifter mode and so move lifter UP
    			ismanualLifterMode = true;
    			lifter.manualUp(shooter.getRawAxis(Rtrigger));
    		} else if(ismanualLifterMode){
    			// no triggers pulled so stop lifter
    			lifter.manualStop();
    			// note we stay in manual lifter mode
    		}
    		*/

    		// NEW Manual lifter controls -- (Shooter left & right triggers)

    		if (manualLifting && shooter.getRawAxis(Rtrigger) < 0.2 && shooter.getRawAxis(Ltrigger) < 0.2) {
    			// no triggers pulled but was pulled previously, so stop lifter
    			lifter.changeTargetHeight(lifter.getCurrentHeight());
    			lifter.leftActuator.set(0);
    			lifter.rightActuator.set(0);
    			manualLifting=false;
			} else if((shooter.getRawAxis(Rtrigger) > 0.2) && (shooter.getRawAxis(Ltrigger) > 0.2)){
    			// both triggers pulled: go into manual lifter mode and stop lifter
				lifter.changeTargetHeight(lifter.getCurrentHeight());
    			lifter.leftActuator.set(0);
    			lifter.rightActuator.set(0);
				manualLifting=false;
    		} else if(!manualLifting && shooter.getRawAxis(Ltrigger) > 0.2){
    			// LT pulled: go into manual lifter mode and so move lifter DOWN 
    			manualLifting=true;
    			lifter.changeTargetHeight(0);  // head towards lowest position //TODO: CALIBRATE
    		} else if(!manualLifting && shooter.getRawAxis(Rtrigger) > 0.2){
    			// RT pulled: go into manual lifter mode and so move lifter UP
    			manualLifting=true;
    			lifter.changeTargetHeight(99); // head towards highest position //TODO: CALIBRATE
    		}

    		
    		
    		/*******************
    		 * Shared Controls *
    		 *******************/
    		
    		if(driver.isAPushed() || shooter.isAPushed()){
    			// A Button: Lower lifter to the ground
    			lifter.changeTargetHeight(0);  //TODO: CALIBRATE
    			ismanualLifterMode = false;
    		} else if(driver.isBPushed() || shooter.isBPushed()){
    			// B Button: move lifter to transport height ~4 in off ground
    			if(ismanualLifterMode){
    				lifter.changeTargetHeight(lifter.getCurrentHeight());
    			}//TODO: CALIBRATE
    			lifter.relativeChangeTargetHeight(2);
    			//	lifter.changeTargetHeight(4);  //TODO: CALIBRATE
    			ismanualLifterMode = false;
    		} else if(driver.isXPushed() || shooter.isXPushed()){
    			// X Button: move lifter to can-pickup height
    			lifter.changeTargetHeight(9);  //TODO: CALIBRATE
    			ismanualLifterMode = false;
    		} else if(driver.isYPushed() || shooter.isYPushed()){
    			//Y Button: unhook stack
    			if(ismanualLifterMode){
    				lifter.changeTargetHeight(lifter.getCurrentHeight());
    			}
    			lifter.relativeChangeTargetHeight(-2); // TODO: Calibrate
    			ismanualLifterMode = false;
    		}

    		//flip tote and right can
    		if(driver.isStartPushed() || shooter.isStartPushed() || shooter.isRStickPushed()){
    			//right can
    		} else if(driver.isBackPushed() || shooter.isBackPushed() || shooter.isLStickPushed()){
    			//flip tote
    		}
    		
    		// If NOT in manual lifter mode then keep lifter moving towards its target
    		SmartDashboard.putBoolean("Lifter Manual:", ismanualLifterMode);
    		if(ismanualLifterMode == false) lifter.goToHeightOffGround();
    		
    		Timer.delay(0.005);
    		driver.clearButtons();
    		shooter.clearButtons();
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    	if(isTest() && isDisabled()){
    		lifter.shutdownCounter = 0;
    	}
    	while (isTest() && isEnabled()){
        	
    		if((shooter.getRawAxis(Rtrigger) > 0.2) && (shooter.getRawAxis(Ltrigger) > 0.2)){
    			ismanualLifterMode = true;
    			lifter.manualStop();
    		} else if(shooter.getRawAxis(Ltrigger) > 0.2){
    			ismanualLifterMode = true;
    			lifter.manualDown(shooter.getRawAxis(Ltrigger));
    		} else if(shooter.getRawAxis(Rtrigger) > 0.2){
    			ismanualLifterMode = true;
    			lifter.manualUp(shooter.getRawAxis(Rtrigger));
    		} else if(ismanualLifterMode){
    			lifter.manualStop();
    		}
    		indexer.wheelControl(shooter.getRawAxis(LY), shooter.getRawAxis(RY));
    		indexer.controlArms(shooter.getRawAxis(LX));
    		
    		//swerve.move(driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX));

    		/* TODO put this in the switch case statement
    		if(driver.isAPushed()){
    			lifter.changeTargetHeight(2.5);
    		} else if(driver.isBPushed()){
    			lifter.changeTargetHeight(6);
    		} else if(driver.isXPushed()){
    			lifter.changeTargetHeight(8);
    		} else if(driver.isYPushed()){
    			lifter.changeTargetHeight(10);
    		}
    		
    		lifter.goToHeightOffGround();
    		
    		if(driver.isLBHeld()){
    			lifter.shutdownCounter = 0;
    		}
    		*/
    		
    		
    		/*
			if (driver.isAHeld()){
            	lifter.moveLeft(1);
            } else if (driver.isBHeld()){
            	lifter.moveRight(1);
            } else if (driver.isYHeld()){
            	lifter.moveRight(-1);
            } else if (driver.isXHeld()){
            	lifter.moveLeft(-1);
            } else {
            	lifter.absoluteStop();
            }*/
			
    		lifter.printPotValues();
    		SmartDashboard.putNumber("inchesOffGroundL", lifter.inchesOffGroundL);
    		SmartDashboard.putNumber("inchesOffGroundR", lifter.inchesOffGroundR);
			/*SmartDashboard.putNumber("Lifter Height", lifter.getCurrentHeight());
			SmartDashboard.putNumber("Lifter Right", shooter.getRawAxis(RX));*/
    	/*	SmartDashboard.putNumber("LeftPotRaw", leftActuator.getAnalogInRaw());
    		SmartDashboard.putNumber("RightPotRaw", rightActuator.getAnalogInRaw());*/
    	/*	
    		SmartDashboard.putBoolean("LeftFWD", leftActuator.isFwdLimitSwitchClosed());
    		SmartDashboard.putBoolean("LeftREV", leftActuator.isRevLimitSwitchClosed());
    		SmartDashboard.putBoolean("RightFWD", rightActuator.isFwdLimitSwitchClosed());
    		SmartDashboard.putBoolean("RightREV", rightActuator.isRevLimitSwitchClosed());*/
    	/*	
    		SmartDashboard.putNumber("Target", lifterTarget);
    		
    		SmartDashboard.putNumber("LeftHeight", getLeftActuatorLength());
    		SmartDashboard.putNumber("RightHeight", getRightActuatorLength());
    		*/
    		
    		/*TODO CALIBRATE TEST 16 SWITCH
    		index = 0;
        	if(ones.get()){
        		index += 1;
        	}
        	if(twos.get()){
        		index += 2;
        	}
        	if(fours.get()){
        		index += 4;
        	}
        	if(eights.get()){
        		index += 8;
        	}
        	System.out.println(index);
    		switch(index){
    		case 0://swerve drive
        		if(driver.isLBHeld()){
        			swerve.setSpeedMode("turbo");
        		} else if(driver.isRBHeld()){
        			swerve.setSpeedMode("sniper");
        		} else{
        			swerve.setSpeedMode("normal");
        		} 
        		
        		//Switching Driving modes
        		if(driver.isRStickHeld()){
        			rotateRadius = 0.0;
        			swerve.isFieldCentric = false;
        			System.out.println("In Robot Centric");
        		} else if(driver.isLStickPushed()){
        			System.out.println("In Field Centric");
        			swerve.isFieldCentric = true;
        			rotateRadius = 0.0;
        		} else if(driver.getRawAxis(Rtrigger) > 0.2){
        			System.out.println("In Object Centric");
        			rotateRadius = objectDistance;
        			swerve.isFieldCentric = false;
        		} else if(driver.getRawAxis(Ltrigger) > 0.2){
        			System.out.println("In Stack Centric");
        			rotateRadius = stackDistance;
        			swerve.isFieldCentric = false;
        		}
        		
        		swerve.swerveControl(driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX), rotateRadius);

    		case 1://lifter Calibration mode
    			if (driver.isAHeld()){
                	lifter.moveLeft(1);
                } else if (driver.isBHeld()){
                	lifter.moveRight(1);
                } else if (driver.isYHeld()){
                	lifter.moveRight(-1);
                } else if (driver.isXHeld()){
                	lifter.moveLeft(-1);
                } else {
                	lifter.absoluteStop();
                }
    			break;
    		case 2://lifter target Control
                if (driver.isAHeld()){
                	lifter.absoluteRaise();
                } else if (driver.isBHeld()){
                	lifter.absoluteLower();
                } else if (driver.isYHeld()){
                	lifter.moveRight(-1);
                } else if (driver.isXHeld()){
                	lifter.moveLeft(-1);
                } else {
                	lifter.absoluteStop();
                }
    			break;
    		case 3: //autoLift
    			if (driver.isAPushed()){
    				lifter.changeTarget(4);
    				lifter.threadedGoToPos(isEnabled());
    			} else if (driver.isBPushed()){
    				lifter.changeTarget((6));
    				lifter.threadedGoToPos(isEnabled());
    			} else if (driver.isXPushed()){
    				lifter.changeTarget(lifter.lifterTarget - 1);
    				lifter.threadedGoToPos(isEnabled());
    			} else if (driver.isYPushed()){
    				lifter.changeTarget(lifter.lifterTarget + 1);
    				lifter.threadedGoToPos(isEnabled());
    			}
    			break;
    		case 4: //indexer
    			indexer.controlArms(driver.getRawAxis(LX));
    			indexer.wheelControl(driver.getRawAxis(LY), driver.getRawAxis(RY));
    			break;
    		case 5:
    			break;
    		case 6:
    			break;
    		case 7:
    			break;
    		case 8:
    			break;
    		case 9:
    			break;
    		case 10:
    			break;
    		case 11:
    			break;
    		case 12:
    			break;
    		case 13:
    			break;
    		case 14:
    			break;
    		case 15:
    			break;
    		default:
    			//We should never get here
    	}*/
    		//indexer.wheelControl(stick1.getRawAxis(LY), stick1.getRawAxis(RY));
    		//System.out.println("POV" + stick1.getPOV());
    		/*
    		if (stick1.isAHeld()){
    			swerve.FRWheel.targetAngle += 5;
    		} else if (stick1.isBHeld()){
    			swerve.FLWheel.targetAngle -= 5;
    		}
    		
				
    		double encoderFR = swerve.FRWheel.rotateMotor.getEncPosition();
    		*/
    		
            boolean is_calibrating = imu.isCalibrating();
            if ( first_iteration && !is_calibrating ) {
                Timer.delay( 0.3 );
                imu.zeroYaw();
                first_iteration = false;
            }
            
            //swerve.test();

            
            
     
            
            SmartDashboard.putBoolean("Ones: ", ones.get());
            SmartDashboard.putBoolean("Twos: ", twos.get());
            SmartDashboard.putBoolean("Fours: ", fours.get());
            SmartDashboard.putBoolean("Eights: ", eights.get());
            
    		/*
            SmartDashboard.putBoolean(  "IMU_Connected",        imu.isConnected());
            SmartDashboard.putBoolean(  "IMU_IsCalibrating",    imu.isCalibrating());
            SmartDashboard.putNumber(   "IMU_Yaw",              imu.getYaw());
            SmartDashboard.putNumber(   "IMU_Pitch",            imu.getPitch());
            SmartDashboard.putNumber(   "IMU_Roll",             imu.getRoll());
            SmartDashboard.putNumber(   "IMU_CompassHeading",   imu.getCompassHeading());
            SmartDashboard.putNumber(   "IMU_Update_Count",     imu.getUpdateCount());
            SmartDashboard.putNumber(   "IMU_Byte_Count",       imu.getByteCount());

            // If you are using the IMUAdvanced class, you can also access the following
            // additional functions, at the expense of some extra processing
            // that occurs on the CRio processor
            
            SmartDashboard.putNumber(   "IMU_Accel_X",          imu.getWorldLinearAccelX());
            SmartDashboard.putNumber(   "IMU_Accel_Y",          imu.getWorldLinearAccelY());
            SmartDashboard.putBoolean(  "IMU_IsMoving",         imu.isMoving());
            SmartDashboard.putNumber(   "IMU_Temp_C",           imu.getTempC());*/
            
            /*
            if(haveRun != true){
            	swerve.FRWheel.goToHome();
            	haveRun = true;
            }
            
            
            swerve.FRWheel.calibration(stick1.isAPushed());*/
            
            //CENTRICITY Control aka switching robot modes
            
            /* SWERVE CODE
            
            if(stick1.isLStickPushed()){
            	swerve.switchToFieldCentric();
            }
            if(stick1.isRStickPushed()){
            	swerve.switchToObjectCentric();
            }
            if(stick1.isStartPushed()){
            	swerve.switchToRobotCentric();
            }
            
            */
            //swerve.changeOrientation(driver.isYPushed(), driver.isBPushed(), driver.isAPushed(), driver.isXPushed());
            //swerve.move(-driver.getRawAxis(LY), driver.getRawAxis(LX), driver.getRawAxis(RX));
            
            
            /*SmartDashboard.putNumber("Back Left Current Encoder Reading", swerve.BLWheel.rotateMotor.getEncPosition());
            SmartDashboard.putNumber("Front Left Current Encoder Reading", swerve.FLWheel.rotateMotor.getEncPosition());
            SmartDashboard.putNumber("Back Right Current Encoder Reading", swerve.BRWheel.rotateMotor.getEncPosition());
            SmartDashboard.putNumber("Front Right Current Encoder Reading", swerve.FRWheel.rotateMotor.getEncPosition());
            
            SmartDashboard.putNumber("TargetAngleFL: ", swerve.FLWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleFL", swerve.FLWheel.currentAngle);
            
            SmartDashboard.putNumber("TargetAngleFR: ", swerve.FRWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleFR: ", swerve.FRWheel.currentAngle);
            
            SmartDashboard.putNumber("TargetAngleBR: ", swerve.BRWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleBR: ", swerve.BRWheel.currentAngle);
            
            SmartDashboard.putNumber("TargetAngleBL: ", swerve.BLWheel.targetAngle);
            SmartDashboard.putNumber("CurrentAngleBL: ", swerve.BLWheel.currentAngle);
            
            SmartDashboard.putNumber("SpeedFL: ", swerve.FLWheel.speed);
            SmartDashboard.putNumber("SpeedFR: ", swerve.FRWheel.speed);
            SmartDashboard.putNumber("SpeedBL: ", swerve.BLWheel.speed);
            SmartDashboard.putNumber("SpeedBR: ", swerve.BRWheel.speed);*/
            
            //SmartDashboard.putBoolean("fwdLimit", actuator.isFwdLimitSwitchClosed());
            //SmartDashboard.putBoolean("RevLimit", actuator.isRevLimitSwitchClosed());
            Timer.delay(0.01);

            /*if (stick1.isAPushed()){
            	twoTalon.set(20000);
            } else if (stick1.isBPushed()){
            	twoTalon.set(15000);
            }*/
            
            
            

            //indexer.controlArms(driver.getRawAxis(LX));
            /*
            if (driver.isAPushed()){
            	lifter.changeTarget(3);
            } else if (driver.isBPushed()){
            	lifter.changeTarget(5);
            } else if (driver.isXPushed()){
            	lifter.changeTarget(lifter.lifterTarget - 1);
            } else if (driver.isYPushed()){
            	lifter.changeTarget(lifter.lifterTarget + 1);
            }
            
            lifter.goToLength();*/
            
            /*if (driver.isAPushed()){
            	lifter.threadedGoToPos(4, isEnabled());
            } else if (driver.isBPushed()){
            	lifter.threadedGoToPos(6, isEnabled());
            }*/
            
            
            lifter.printPotValues();
            driver.clearButtons();
            shooter.clearButtons();
    	}
    }
    

}
