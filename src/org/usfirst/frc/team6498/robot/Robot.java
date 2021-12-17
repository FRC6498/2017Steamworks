package org.usfirst.frc.team6498.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	final String[] autoMode = {"StraightRight", "StraightLeft",
			                   "ClockWise", "CounterClockWise"};
	double dashDataSlider0;
	int autoIndex;
	
	TurnTarget findTurnTarget;
	StraightTarget findStraightTarget;
	double turn;
		
	Spark frontLeftMotor = new Spark(0);
	Spark rearLeftMotor = new Spark(1);
	Spark frontRightMotor = new Spark(2);
	Spark rearRightMotor = new Spark(3);
	
	Spark climber = new Spark(4);
	
	static DigitalInput ReflectiveR = new DigitalInput(2);
    static DigitalInput ReflectiveL = new DigitalInput(3);
    static DigitalInput GearDetector = new DigitalInput(4);
    
    static Encoder encoder = new Encoder(0,1,true, Encoder.EncodingType.k2X);
    
    Timer timer = new Timer();
    int interruptTick=0;
       
	RobotDrive myRobot = new RobotDrive(frontLeftMotor, rearLeftMotor, 
			                            frontRightMotor, rearRightMotor);
	
	
	Joystick stick = new Joystick(0);
	JoystickButton button1 = new JoystickButton(stick,1);
	JoystickButton button2 = new JoystickButton(stick,2);
	boolean climberRun=false;
	boolean climberFlag=false;
	double rampIndex;
	
	/* type variables for autonomous mode */
	final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//System.out.println("robot init ----- ");
		//UsbCamera camera0 = CameraServer.getInstance().startAutomaticCapture();
		//UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture();
		//camera0.setResolution(320, 240);
		//camera1.setResolution(320, 240);
		
		SmartDashboard.putString("DB/String 5", "0 = StraightRight");
		SmartDashboard.putString("DB/String 6", "1 = StraightLeft");
		SmartDashboard.putString("DB/String 7", "2 = ClockWise");
		SmartDashboard.putString("DB/String 8", "3 = CounterClockWise");	
		
		gyro.calibrate();
		gyro.reset();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
		System.out.println("autonomous init time = " + timer.get());
		/*
    	encoder.setMaxPeriod(.1);
    	encoder.setMinRate(10);
    	encoder.setDistancePerPulse(5);
    	encoder.setReverseDirection(false);
    	encoder.setSamplesToAverage(7);
    	*/
			
		dashDataSlider0=SmartDashboard.getNumber("DB/Slider 0", 0.0);
		if( dashDataSlider0 >3.9){
			dashDataSlider0=0;
		}
		autoIndex = (int)dashDataSlider0;
		System.out.println("autonomous " + autoMode[autoIndex]);
		switch(autoMode[autoIndex]){
    	    case "CounterClockWise":
    	    case "ClockWise":
    	    	findTurnTarget= new TurnTarget();
    	    break;	
    	    
    	    case "StraightRight":
    	    	turn = 1.0; // after backing up turn is to the right
    	    	findStraightTarget = new StraightTarget();
    	    break;	
    	    
    	    case "StraightLeft":
    	    	turn = -1.0; // after backing up turn is to the left
    	    	findStraightTarget = new StraightTarget();
    	    break;
    	}
		myRobot.setSafetyEnabled(false);
    	gyro.reset();
    	encoder.reset();
    	    	    	    	
	}

	/*
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		if((interruptTick++)%50 == 0)
			System.out.println("time= " + timer.get());
		
		switch(autoMode[autoIndex]){
		    
		    case "CounterClockWise":
		    case "ClockWise":
		    	findTurnTarget.TurnToTarget(myRobot, gyro, autoMode[autoIndex], ReflectiveR, 
		    			                    ReflectiveL, encoder, GearDetector);
		        break;
		        
		    case "StraightRight":
		    case "StraightLeft":	
		    	findStraightTarget.StraightToTarget(myRobot, gyro, encoder, turn, GearDetector);
		        break;
		    }
		}


	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
		myRobot.setSafetyEnabled(true);
		rampIndex=.3;
		System.out.println("telopInit --");
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		//if((interruptTick++)%25 == 0)
			//System.out.println("time= " + timer.get());
		
		myRobot.arcadeDrive(stick);
		
		if( button1.get() == true){
			climberFlag = true;
		}
		if( button2.get() == true){
			climberFlag = false;
		}
		
		
		if (climberFlag == true){
			telopRampClimber();
		}else{
			climber.setSpeed(0);
		}
		
		
		//System.out.println(encoder.get());
	}
	
	void telopRampClimber(){
		rampIndex = rampIndex+.03;
		if(rampIndex>=1.0)
			rampIndex=1.0;
		climber.setSpeed(rampIndex);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		//System.out.println("test periodic");
		myRobot.setSafetyEnabled(true);
		LiveWindow.run();
		
		if(GearDetector.get() == true){
			System.out.println("GEAR PULLED");
		}else{
			System.out.println("gear in");
		}
		
		
		
		if(SmartDashboard.getBoolean("DB/Button 0", false)){
			SmartDashboard.putBoolean("DB/LED 0", true);
			System.out.println("Tape seen: ");
	
		}else{
			SmartDashboard.putBoolean("DB/LED 0", false);
		}
		if (ReflectiveL.get()==true){
			SmartDashboard.putBoolean("DB/LED 1", true);
		}else{
			SmartDashboard.putBoolean("DB/LED 1", false);
		}
		if (ReflectiveR.get()==true){
			SmartDashboard.putBoolean("DB/LED 2", true);
		}else{
			SmartDashboard.putBoolean("DB/LED 2", false);
		} 
		if(SmartDashboard.getBoolean("DB/Button 3", false)){
			SmartDashboard.putBoolean("DB/LED 3", true);
			climber.setSpeed(1);
		}
		else{
			climber.setSpeed(0);
			}
			
		
	}
}

