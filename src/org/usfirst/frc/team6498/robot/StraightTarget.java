package org.usfirst.frc.team6498.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class StraightTarget {
   static final double FORWARD_ENCODER_PULSES = 1519; //1519
   static final double REVERSE_ENCODER_PULSES = 600;
   static final double AFTER_TURN_FORWARD_ENCODER_PULSES = 1200;
   
   static final double TURN_RATE_NEG =-.54;
   static final double TURN_RATE=.54;
   
   String autoState = "forward";
   Timer timer1 = new Timer();
   
   void StraightToTarget(RobotDrive _myRobot, ADXRS450_Gyro _gyro, Encoder _encoder, 
		                 double _turn, DigitalInput _GearDetector ){
      	    
	  switch(autoState){
	     case "forward":
	  	    if(Math.abs(_encoder.get()) < FORWARD_ENCODER_PULSES){
    	       double angle = _gyro.getAngle();
    	       _myRobot.drive(-.5, -angle*.03);
    	    }else {
    	 	      _myRobot.drive(-.3, 0);
    	 	    // autoState = "waitTime";
	 	         //timer1.start();
	 	         
    	 	      if(_GearDetector.get() == true){
    	 	    	  System.out.println("GONE forward");
    	 	         autoState = "waitTime";
    	 	         timer1.start();
    	 	      }
    	 	}
	  	     
		 break; 
		 
	     case "waitTime": // time for gear to clear robot
	        if(timer1.get()< .75 ){
	        	_myRobot.drive(-.25, 0);
	        	if(_GearDetector.get() == false){
			    	   autoState="forward";
			    	   System.out.println("Gear Lost!");
			       }
	        }else{
	        	System.out.println("GEAR PULLED");
	        	autoState = "backUp";
	        	_encoder.reset();
	        	_myRobot.drive(.5,0);
	        }
	     break;	 
		 
		 case "backUp":
			if(Math.abs(_encoder.get()) < REVERSE_ENCODER_PULSES){
				_myRobot.drive(.5,0);	
			}else{
				System.out.println("BACKED UP");
				_gyro.reset();
				autoState = "firstTurn";
				_myRobot.tankDrive(TURN_RATE_NEG*_turn, TURN_RATE*_turn); // clock wise if _turn 1
			}
		 break;
		 
		 case "firstTurn":
			 double angle = _gyro.getAngle();
			 if(Math.abs(angle) < 88){
				 System.out.println("angle = "+angle);
				 _myRobot.tankDrive(TURN_RATE_NEG*_turn, TURN_RATE*_turn); 
			 }else{
				 System.out.println("angle = "+angle);
				 System.out.println("firstTurn");
				 _encoder.reset();
				 _myRobot.drive(-.6,0); // go straight
				 autoState = "secondForward";	 
			 }
		 break;	 
		 
		 case "secondForward":
			 if(Math.abs(_encoder.get()) < AFTER_TURN_FORWARD_ENCODER_PULSES){
					_myRobot.drive(-.5,0);	
				}else{
					System.out.println("secondForward");
					_gyro.reset();
					autoState = "secondTurn";
					_myRobot.tankDrive(TURN_RATE*_turn, TURN_RATE_NEG*_turn); //  counter clock wise if _turn 1
				}
		 break;	 
		 
		 case "secondTurn":
			 double angle2 = _gyro.getAngle();
			 if(Math.abs(angle2) < 85){
				 _myRobot.tankDrive(TURN_RATE*_turn, TURN_RATE_NEG*_turn); 	 
			 }else{
				 System.out.println("secondTurn & going forward");
				 _myRobot.drive(-.6,0); // go forward
			 }
      }//switch
   }// method   
} // class
