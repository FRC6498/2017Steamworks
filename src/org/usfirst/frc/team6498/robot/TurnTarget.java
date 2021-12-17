package org.usfirst.frc.team6498.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class TurnTarget {
	  double straightEncoderPulses;  // variable to stop when driving straight
	  double afterTurnEncoderPulses;
	  
	  static final double CLOCK_WISE_STRAIGHT = 1508;
	  static final double CLOCK_WISE_AFTER_TURN=804;
	  	  
	  static final double COUNTER_CLOCK_WISE_STRAIGHT = 1508;
	  static final double COUNTER_CLOCK_WISE_AFTER_TURN=804;
	  	  
	  double tankDriveNonSeen; //arguments in parameters list
	  double tankDriveOneSeen;
	  
	  static final double TANK_DRIVE_NONSEEN=.54; // parameter values for turn .52
	  static final double TANK_DRIVE_ONESEEN=.50; //.56
	  
	  String autoState= "forward";
	  
	  double lastEncoder=0;
	  Timer timer2 = new Timer();
	  	  
	  void TurnToTarget(RobotDrive _myRobot, ADXRS450_Gyro _gyro, String _turn, 
			            DigitalInput _ReflectiveR, DigitalInput _ReflectiveL, 
			            Encoder _encoder, DigitalInput _GearDetector){
		
		  if(_turn == "ClockWise"){
			straightEncoderPulses=CLOCK_WISE_STRAIGHT;
			afterTurnEncoderPulses=CLOCK_WISE_AFTER_TURN;
			
			tankDriveNonSeen = -TANK_DRIVE_NONSEEN;
			tankDriveOneSeen = -TANK_DRIVE_ONESEEN;
			
		}else{
			straightEncoderPulses=COUNTER_CLOCK_WISE_STRAIGHT;
			afterTurnEncoderPulses=COUNTER_CLOCK_WISE_AFTER_TURN;
			
			tankDriveNonSeen = TANK_DRIVE_NONSEEN;
			tankDriveOneSeen = TANK_DRIVE_ONESEEN;
		}
			
		switch(autoState){
	
		case "forward":
			if(Math.abs(_encoder.get()) < straightEncoderPulses){
	    	    double ang1 = _gyro.getAngle();
	    	    _myRobot.drive(-.5, -ang1*.03);
	    	    //System.out.println(Math.abs(encoder.get()));
	    	    
	    	}else{
	    		
	    		System.out.println(Math.abs(_encoder.get()) + "turn EncoderGyroDriveStraight");
	    		//_myRobot.drive(0, 0);
	    		_myRobot.tankDrive(tankDriveOneSeen, -tankDriveOneSeen);
	    		autoState = "noneSeen";
	    		
	    	} 
		break;
		
		case "noneSeen":
			//turn counterclockwise FIRST parameter + and SECOND parameter -
			_myRobot.tankDrive(tankDriveNonSeen, -tankDriveNonSeen);
			if(_ReflectiveL.get() || _ReflectiveR.get()){
				System.out.println("ONE SEEN");
				_myRobot.tankDrive(tankDriveOneSeen, -tankDriveOneSeen);
				_encoder.reset();
				autoState="oneSeen";
			}
	    break;		
		
		case "oneSeen":
			//continues to turn counterclockwise at speed after one tape seen
			// when left and right reflector seen robot drives straight
			lastEncoder = Math.abs(_encoder.get());
			if(lastEncoder < 15){
				tankDriveOneSeen = tankDriveOneSeen +.02;
				_myRobot.tankDrive(tankDriveOneSeen, -tankDriveOneSeen);
				System.out.println("speed up turn");
			}
			if(_ReflectiveL.get() && _ReflectiveR.get()){
				//_myRobot.tankDrive(0, 0);
				_gyro.reset();
				_encoder.reset();
				double ang2 = _gyro.getAngle();
			    _myRobot.drive(-.4, -ang2*.03);
  			    autoState="deliver";
  			    System.out.println("TWO SEEN");
			}
		break;	
		
		case "deliver":
			//drives Robot forward continuous
			
			if(Math.abs(_encoder.get()) < afterTurnEncoderPulses){
				double angle = _gyro.getAngle();
			    _myRobot.drive(-.4, -angle*.03);
			}else{
				_myRobot.drive(-.3, 0);
				autoState = "gearPulledOut";
			}	
		break;
		
		case "gearPulledOut":
			_myRobot.drive(-.25, 0);
			if(_GearDetector.get() == true){
				System.out.println("PULL GEAR");
				timer2.start();
				autoState = "waitTime";
			}
			//timer2.start();
			//autoState = "waitTime";
		break;	
		
		case "waitTime": // time for gear to clear robot
	       if((timer2.get() < .75)){
	    	   _myRobot.drive(-.25, 0);
		       if(_GearDetector.get() == false){
		    	   autoState="gearPulledOut";
		    	   System.out.println("Gear Lost!");
		       }
		   }else{
			   _encoder.reset();
			   _myRobot.drive(.5, 0); // backing up
			   autoState = "backUp";	
		   }
		break;
	    
		case "backUp":
			if(Math.abs(_encoder.get()) < afterTurnEncoderPulses){
				_myRobot.drive(.5, 0); // backing up   
			}else{
				_gyro.reset();
				_myRobot.tankDrive(-tankDriveNonSeen, tankDriveNonSeen); // clock wise or right
				autoState = "turn";	
			}
		break;
		
		case "turn":
			double angle = _gyro.getAngle();
			 if(Math.abs(angle) < 44){
				 _myRobot.tankDrive(-tankDriveNonSeen, tankDriveNonSeen);
			 }else{
				 _myRobot.drive(-.46,0); // go straight
				 autoState = "secondForward"; 
			 }
		break;
		
		case "secondForward":
			 _myRobot.drive(-.46, 0);
		 break;	
		} // switch
		
	  }// method
} //class
