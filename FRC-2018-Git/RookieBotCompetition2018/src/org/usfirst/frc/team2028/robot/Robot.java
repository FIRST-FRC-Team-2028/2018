package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
//import com.ctre.CANTalon;
//import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDController;

//import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Robot extends IterativeRobot {
	//	 static ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
	Ramp ramp;
	static Gyro gyro = new Gyro();
	DigitalInput Dig1 = new DigitalInput(0);
	PIDController pidController;
	PIDController cameraController;
	XboxController Controller = new XboxController(0);		//FIXME make C lowercase
	//	 XboxController WhiteController = new XboxController(4); 
	PixyCamera pixycamera = new PixyCamera();
	Joystick LaunchPad = new Joystick(1);
	Joystick pots = new Joystick(2);
	Joystick Switches = new Joystick(3);
	Ultrasonic ultrasonic = new Ultrasonic();
	Compressor comp;
	DriveToPositionAuto drivetopos;
	AutoOption1 autooption1;
	AutoOption2 autooption2;
	AutoOption3 autooption3;
	AutoOption4 autooption4;
	AutoOption5 autooption5;
	AutoOption6 autooption6;
	AutoOption7 autooption7;
	JoystickDrive joystickdrive = null;
	
	
	AnalogInput sonar;
	//	 AnalogInput camera_x;
	AnalogInput sidesonar;
	AnalogInput tank;

	Timer myTimer;

	private double delay;
	private int positionknob;
	private int objectiveknob;
	private int delayknob;
	private double setpoint;
	private double inc;
	private Lift lift;
	private Drive drive;
	private Gripper gripper;
	private LineCamera lineCamera;
	//	double P = Parameters.Pid.CAMERA.getP();
	private double  Auto_Selection = 0;
	private int i = 0;
	private double testcount;
	private boolean leftswitch;
	private boolean leftscale;
	private boolean gamedatavalid;
	// 	 
	//     final int  CAN_ID_Left_Master_Top = 10;
	//	 final int  CAN_ID_Left_Follower_Bottom = 11;  
	//	 final int  CAN_ID_Right_Master_Top = 20; 
	//	 final int  CAN_ID_Right_Follower_Bottom = 21; 
	//	 final int  CAN_ID_Test_Talon = 30;
	//	 
	//	 public static WPI_TalonSRX Right_Drive_Master;
	//	 public static WPI_TalonSRX Right_Drive_Follower; 
	//	 public static WPI_TalonSRX Left_Drive_Master;
	//	 public static WPI_TalonSRX Left_Drive_Follower;
	//	 public static WPI_TalonSRX Test_Talon;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		lift = new Lift();
		joystickdrive = new JoystickDrive(Controller);
		drive = new Drive(joystickdrive);
		ramp = new Ramp();
		gripper = new Gripper();
		pidController = new PIDController(Parameters.Pid.CONTROLLER.getP() , Parameters.Pid.CONTROLLER.getI() 
				, Parameters.Pid.CONTROLLER.getD(), Parameters.Pid.CONTROLLER.getF() , gyro , drive);
		pidController.setOutputRange(-1000, 1000);
		cameraController = new PIDController(Parameters.Pid.CAMERA.getP(), Parameters.Pid.CAMERA.getI(), Parameters.Pid.CAMERA.getD(), 
				Parameters.Pid.CAMERA.getF(), pixycamera, drive);
		cameraController.setOutputRange(-3000, 3000);
		cameraController.setInputRange(-1.65, 1.65);
		drivetopos = new DriveToPositionAuto(pidController, drive, 24.0, 20 , ultrasonic);
		//		 gyro.reset();



		drive.setPTOHigh();

		setpoint = Parameters.ZERO_POSITION;
		//		 Right_Drive_Master = new WPI_TalonSRX(CAN_ID_Right_Master_Top);

		//		 Right_Drive_Master.set(ControlMode.Velocity, 0);
		//		 Right_Drive_Master.setNeutralMode(NeutralMode.Brake);
		//
		////		 Right_Drive_Master.enable();
		//		 
		//		 Right_Drive_Follower = new WPI_TalonSRX(CAN_ID_Right_Follower_Bottom);
		//		 Right_Drive_Follower.set(ControlMode.Follower, CAN_ID_Right_Master_Top);
		//		 Right_Drive_Follower.setNeutralMode(NeutralMode.Brake);
		//		 Right_Drive_Follower.follow(Right_Drive_Master);
		//		 
		//		 Left_Drive_Master = new WPI_TalonSRX(CAN_ID_Left_Master_Top);
		//		 Left_Drive_Master.set(ControlMode.Velocity, 0);
		//		 Left_Drive_Master.setNeutralMode(NeutralMode.Brake);
		//
		////		 Left_Drive_Master.enable();
		//
		//		 Left_Drive_Follower = new WPI_TalonSRX(CAN_ID_Left_Follower_Bottom);
		//		 Left_Drive_Follower.set(ControlMode.Follower, CAN_ID_Left_Master_Top);
		//		 Left_Drive_Follower.setNeutralMode(NeutralMode.Brake);
		//		 Left_Drive_Follower.follow(Left_Drive_Master);		 

		//		 Test_Talon = new WPI_TalonSRX(CAN_ID_Test_Talon);
		////		 Test_Talon.setNeutralMode(NeutralMode.Brake);
		//		 Test_Talon.set(ControlMode.Position, 0);
		//
		//		 
		//			Test_Talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		//			Test_Talon.config_kP(0, 9, 0);
		//			Test_Talon.config_kI(0, 0.0002, 0);
		//			Test_Talon.config_kD(0, 0, 0);

		//		Right_Drive_Master.setSelectedSensorPosition(0, 0, 0);
		//		Right_Drive_Master.config_kP(0, 0.8, 0);
		//		Right_Drive_Master.config_kI(0, 0.003, 0);
		//		Right_Drive_Master.config_kD(0, 0.001, 0);
		//		Right_Drive_Master.config_kF(0, 0, 0);
		//		Right_Drive_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//		Right_Drive_Master.setSensorPhase(false);
		//		Right_Drive_Master.setInverted(true);
		//		Right_Drive_Follower.setInverted(true);

		//		Left_Drive_Master.setSelectedSensorPosition(0, 0, 0);
		//		Left_Drive_Master.config_kP(0, 0.8, 0);
		//		Left_Drive_Master.config_kI(0, 0.003, 0);
		//		Left_Drive_Master.config_kD(0, 0.001, 0);
		//		Left_Drive_Master.config_kF(0, 0, 0);
		//		Left_Drive_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		//		Left_Drive_Master.setSensorPhase(true);
		//		Left_Drive_Master.setInverted(false);	

		//	 	 sonar    = new AnalogInput(0);
		//		 camera_x = new AnalogInput(1);
		sidesonar = new AnalogInput(2);
		tank = new AnalogInput(3);
		if(Parameters.COMPRESSOR_AVAILABLE){
			comp = new Compressor();
		}
		myTimer = new Timer();
	}


	public int decodeRotary(int axisnumber)
	{
		//		double X = pots.getRawAxis(0);
		//		double m = X*10;
		//		int num = (int)m;
		//		double y = (0.9)*num + 6;
		//		SmartDashboard.putNumber("switch position", y);
		//		return y;
		double[] Rotary_Switch_1 = { -1.000,       //  1 
				-0.800,       //  2
				-0.600,       //  3
				-0.400,       //  4
				-0.200,       //  5 
				-0.000,       //  6
				0.200,       //  7 
				0.400,       //  8
				0.600,       //  9 
				0.800,       // 10 
				1.000   };   // 11

		double Rotary_Switch_1_Deadband = .05;
		int Rotary_Switch_1_Position = -99;         

		double Rotary_Switch_1_Value = pots.getRawAxis(axisnumber);
		SmartDashboard.putNumber("Switch Raw Value" + axisnumber, Rotary_Switch_1_Value);
		for(int i=0; i<Rotary_Switch_1.length; i++){
			if (Math.abs(Rotary_Switch_1_Value - Rotary_Switch_1[i]) < Rotary_Switch_1_Deadband)
			{ 		
				Rotary_Switch_1_Position = (int)(i + 1);	 
				SmartDashboard.putNumber("Switch Value" + axisnumber, Rotary_Switch_1_Position);  
				return Rotary_Switch_1_Position;
			}

		} // End of for		
		return Rotary_Switch_1_Position;
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() 
	{ 
		positionknob = decodeRotary(0);
		objectiveknob = decodeRotary(1);
		delayknob = decodeRotary(2);


		//		gyro.reset();//TODO: adjust for start orientation angle.
		String gamedata = DriverStation.getInstance().getGameSpecificMessage();
		gamedata = gamedata.toUpperCase();
		drive.resetPosition();
		if(gamedata.length() >= 2)
		{
			gamedatavalid = true;
		}
		if(gamedatavalid && gamedata.charAt(0) == 'L')
		{
			leftswitch = true;
		}else
		{
			leftswitch = false;
		}
		if(gamedatavalid && gamedata.charAt(1) == 'L')
		{
			leftscale = true;
		}else
		{
			leftscale = false;
		}

		switch(delayknob)
		{
		case 1:
			delay = 0.0;
			break;
		case 2:
			delay = 0.5;
			break;
		case 3:
			delay = 1.0;
			break;
		case 4:
			delay = 1.5;
			break;
		case 5:
			delay = 2;
			break;
		case 6:
			delay = 2.5;
			break;
		case 7:
			delay = 3;
			break;
		case 8:
			delay = 3.5;
			break;
		case 9:
			delay = 4;
			break;
		case 10:
			delay = 4.5;
			break;
		case 11:
			delay = 5;
			break;
		default:
			delay = 0;
			break;
		}

		autooption1 = new AutoOption1(pidController, drive, delay, positionknob, leftswitch);
		autooption2 = new AutoOption2(pidController, drive, ultrasonic, delay, positionknob,
				leftswitch, gamedata);
		//		autooption3 = new AutoOption3("fix me");
		autooption4 = new AutoOption4(pidController, drive, ultrasonic, delay, positionknob,
				leftscale, gamedata);
		autooption5 = new AutoOption5(pidController, drive, ultrasonic, delay, positionknob,
				leftswitch, gamedata);
		autooption6 = new AutoOption6(pidController, drive, ultrasonic, delay, positionknob,
				leftscale, gamedata);

		switch(objectiveknob)
		{
		case 1:
			autooption1.start();
			break;
		case 2:
			autooption2.start();
			break;
		case 3:
			autooption3.start();
			break;
		case 4:
			autooption4.start();
			break;
		case 5:
			autooption5.start();
			break;
		case 6:
			autooption6.start();
			break;
		case 7:
			autooption7.start();
			break;
		default:
			break;
		}
		SmartDashboard.putBoolean("autostart", true);
		//This method is called once each time the robot enters autonomous mode
		//		comp.stop();
		//		i = 0;
		//		resetPosition(Left_Drive_Master);
		//		resetPosition(Right_Drive_Master);
		//		gyro.reset();
		//	  Auto_Selection = SmartDashboard.getNumber("Auto_Selection", 10);
		//		
		//	    myTimer.reset();    	// Reset timer to 0 sec
		//	    mymer.start();	    // Start timer
		//
		//	  System.out.println(Auto_Selection); 
		//	  
		//		//
		//		//  This is not the slickest way to decifer a rotaryswitch but 
		//		//
		//		//                                             Switch Position
		//		double[] Rotary_Switch_1 = { -1.000,       //  1 
		//				                     -0.840,       //  2
		//				                     -0.671,       //  3
		//				                     -0.507,       //  4
		//				                     -0.351,       //  5 
		//				                     -0.187,       //  6
		//				                      0.000,       //  7 
		//				                      0.181,       //  8
		//				                      0.385,       //  9 
		//				                      0.590,       // 10 
		//				                      0.795   };   // 11
		//				     
		//      double Rotary_Switch_1_Deadband = .02;
		//      int Rotary_Switch_1_Position = -99;         
		//
		//      double Rotary_Switch_1_Value = LaunchPad.getX();
		//      SmartDashboard.putNumber("Switch Raw Value", Rotary_Switch_1_Value);
		//      for(int i=0; i<Rotary_Switch_1.length; i++){
		//           if (Math.abs(Rotary_Switch_1_Value - Rotary_Switch_1[i]) < Rotary_Switch_1_Deadband)
		//               { 		
		//          	 Rotary_Switch_1_Position = i + 1;	 
		//          	 SmartDashboard.putNumber("Switch Value", Rotary_Switch_1_Position);  
		//          	 }
		//           } // End of for
		//		
		//		// =======================================================================================		  
		//	  
		//      Auto_Selection = Rotary_Switch_1_Position;
		//	  

	}
	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		if(Parameters.COMPRESSOR_AVAILABLE){
			comp.stop();
		}
		Scheduler.getInstance().run();
		//		SmartDashboard.putBoolean("can you see me", true);
		//		SmartDashboard.putBoolean("leftswitch", leftswitch);
		//		DriveToPositionAuto(drive, 10);
		//		Autonomous.leftPositionSwitch(true, drive, pidController);

		//			Autonomous.leftPositionAutoLine(drive);
		//		comp.stop();
		//		// This method is called each time the robot receives a packet instructing the robot to be in autonomous enable mode		
		// 		String gamedata = DriverStation.getInstance().getGameSpecificMessage();
		////        if(gamedata.charAt(0) == 'L')	//This gets Field Management System's (FMS) assignment of
		////    	{								// of our side of the switch and tells the robot
		////    		if(i < 1)					// to go on either the left or right side of the
		////    		{							// switch depending on what the FMS tells us.
		////    			comp.stop();
		////    			Autonomous.leftSwitch(Left_Drive_Master, Right_Drive_Master);
		////    			i +=1;
		////    		}
		////    		else
		////    		{
		////    			Left_Drive_Master.stopMotor();
		////    			Right_Drive_Master.stopMotor();
		////    		}
		////    	}
		////    	else
		////    	{
		////    		if(i < 1)
		////    		{
		////    			comp.stop();
		////    			Autonomous.rightSwitch(Left_Drive_Master, Right_Drive_Master);
		////    			i +=1;
		////    		}
		////    		else
		////    		{
		////    			Left_Drive_Master.stopMotor();
		////    			Right_Drive_Master.stopMotor();
		////    		}
		////    	}
		//        
		////    	if(gamedata.charAt(1)== 'L')
		////    	{//switch position is on the left
		////    	}
		////    	else
		////    	{//switch position is on the right
		////    	}
		//
		//        
		//	if(Auto_Selection == 1.0) {//if the selector switch is on the first option, run autonomous code
		//							   // to drive to the left platform of the switch
		//		
		//			driveGyro(120);
		////			i += 1;
		//		//Autonomous.stop();
		////	    if (myTimer.get() < 2.0)                            {  driveBackward(.25);               }
		////	    if((myTimer.get() > 2.0) && (myTimer.get() < 3.0))  {  driveTurnRight(.25);              }
		////	    if((myTimer.get() > 3.0) && (myTimer.get() < 4.4))  {  driveBackward(.25);               }
		////	    if (myTimer.get() > 4.4)                            {  driveStop();     myTimer.stop();  }
		//	}
		//
		//	if (Auto_Selection == 2) {	
		//	    if (myTimer.get() < 4.5)                            {  driveBackward(.25);               }
		//	    if((myTimer.get() > 4.5) && (myTimer.get() < 5.0))  {  driveTurnLeft(.25);               }
		//	    if((myTimer.get() > 5.0) && (myTimer.get() < 6.5))  {  driveBackward(.25);    	         }	    
		//	    if (myTimer.get() > 6.5)                            {  driveStop();     myTimer.stop();  }      }
		//
		//	if (Auto_Selection == 3) {	
		//	    if (myTimer.get() < 2.5)                            {  driveBackward(.25);               }
		//	    if((myTimer.get() > 2.5) && (myTimer.get() < 5.5))  {  driveTurnLeft(.5);               }    
		//	    if (myTimer.get() > 5.5)                            {  driveStop();     myTimer.stop();  }    }
		//	    
		//	if (Auto_Selection == 4) {	
		//	    if (myTimer.get() < 1.4)                            {  driveForward(.25);               }
		//	    if((myTimer.get() > 1.4) && (myTimer.get() < 2.4))  {  driveTurnLeft(.25);              }
		//	    if((myTimer.get() > 2.4) && (myTimer.get() < 4.4))  {  driveForward(.25);               }
		//	    if (myTimer.get() > 4.4)                            {  driveStop();     myTimer.stop();  }	    }
		//	
		//	if (Auto_Selection == 5) {	
		//	    if (myTimer.get() < 1.5)                            {  driveForward(.25);               }
		//	    if((myTimer.get() > 1.5) && (myTimer.get() < 2.0))  {  driveTurnRight(.25);               }
		//	    if((myTimer.get() > 2.0) && (myTimer.get() < 6.5))  {  driveForward(.25);    	         }	    
		//	    if (myTimer.get() > 6.5)                            {  driveStop();     myTimer.stop();  }      }
		//	if (Auto_Selection == 6) {
		//		driveStop();
		//     }
	}

	// end of autonomousPeridoic  




	double time1;
	double error1;
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {//The teleopInit method is called once each time the robot enters teleop mode 
		//		Right_Drive_Master.enableVoltageCompensation(true);
		//		Left_Drive_Master.enableVoltageCompensation(true);
		testcount = 0;
		drive.resetPosition();
		time1 = Timer.getFPGATimestamp();
		error1 = pidController.getError();
	}

	public static void resetPosition(WPI_TalonSRX motor)
	{
		motor.setSelectedSensorPosition(0, 0, 0);
	}

	static double fred = 0;
	public static double turnTo(double angle)
	{
		double first = gyro.getAngle()/360;
		double n = Math.floor(first);
		double result = n * 360;
		double desired = result + angle;
		double delta = desired - gyro.getAngle();
		if(delta > -180)
		{
			fred = delta - 360;
		}else{
			fred = delta + 360;
		}
		if(Math.abs(delta) < 180)
		{
			return delta + gyro.getAngle();
		}else{
			return fred + gyro.getAngle();
		}
	}
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		SmartDashboard.putNumber("X axis", pots.getRawAxis(0));
		SmartDashboard.putNumber("Y axis", pots.getRawAxis(1));
		SmartDashboard.putNumber("Z axis", pots.getRawAxis(2));

		drive.go();
		if(Parameters.COMPRESSOR_AVAILABLE)
		{
			comp.stop();
		}
		double rightSpeed;
		double leftSpeed;
		SmartDashboard.putBoolean("FwdLimitSwitch", lift.isLiftUp());
		SmartDashboard.putBoolean("RevLimitSwitch", lift.isLiftDown());
		SmartDashboard.putNumber("P VALUE", Parameters.Pid.CAMERA.getP());
		SmartDashboard.putNumber("Right Master out", drive.getVoltage(false, true));
		SmartDashboard.putNumber("Left Master out", drive.getVoltage(true, true));
		SmartDashboard.putNumber("lift position", lift.getPosition());
		double leftStickSpeed = Controller.getRawAxis(1) * -4069 * 500 / 600;
		double rightStickSpeed = Controller.getRawAxis(5) * -4069 * 500 / 600;
		SmartDashboard.putNumber("Left Joystick speed", leftStickSpeed);
		SmartDashboard.putNumber("Right Joystick speed", rightStickSpeed);
		SmartDashboard.putNumber("right position", drive.getrightposition());
		SmartDashboard.putNumber("left position", drive.getleftposition());

		if(LaunchPad.getRawButton(9))
		{
			decodeRotary(0);
			decodeRotary(1);
			decodeRotary(2);
			SmartDashboard.putNumber("adjust pot", pots.getRawAxis(3));
			SmartDashboard.putNumber("Gripper pot", pots.getRawAxis(4));
		}

		if (Math.abs(Controller.getRawAxis(1)) > .1)
		{
			rightSpeed = -Controller.getRawAxis(1);
		}
		else
		{
			rightSpeed = 0;
		}
		if (Math.abs(Controller.getRawAxis(5)) > .1)
		{
			leftSpeed = -Controller.getRawAxis(5);
			leftStickSpeed = Controller.getRawAxis(5) * -4069 * 500 / 600;
		}
		else
		{
			leftSpeed = 0;
			leftStickSpeed = 0;
		}



		//		if(Controller.getRawButton(10))
		//		{
		//			gyro.reset();
		//		}
		SmartDashboard.putNumber("pidcontroller get", pidController.get());
		SmartDashboard.putNumber("PIDGET", gyro.pidGet());

		//		double rotatespeed = 400;
		if(Controller.getPOV() == 270)
		{
			inc += 50;
			lift.setPosition(inc);
		}
		if(Controller.getPOV() == 315)
		{
			inc -= 50;
			lift.setPosition(inc);
		}
		if(Controller.getPOV() == 225)
		{
			lift.zeroPosition();
		}


		if(LaunchPad.getRawButton(5))
		{
			gripper.pickupCube();
		}
		gripper.stopGripper();
		if(LaunchPad.getRawButton(1))
		{
			gripper.dribbleCube();
		}
		if(LaunchPad.getRawButton(2))
		{
			gripper.ejectCube();
		}
		if(LaunchPad.getRawButton(3))
		{
			gripper.launchCube();
		}


		if(LaunchPad.getRawButton(10))
		{
			ramp.retractOutrigger();
		}
		if(LaunchPad.getRawButton(11))
		{
			ramp.deployOutriggers();
		}

		if((Controller.getRawButton(6) && !pidController.isEnabled()) || (Controller.getRawButton(6) && !cameraController.isEnabled()))
		{
			SmartDashboard.putNumber("loopValue", 1);
			drive.rotate(leftStickSpeed);
		} else if ((Controller.getRawButton(5) && !pidController.isEnabled()) || (Controller.getRawButton(5) && !cameraController.isEnabled())) 
		{
			SmartDashboard.putNumber("loopValue", 2);
			drive.set(leftStickSpeed);
		}
		else if(/*!WhiteController.getAButton() && */!pidController.isEnabled() && !cameraController.isEnabled())
		{
			SmartDashboard.putNumber("loopValue", 3);
			drive.setVoltage(leftSpeed, rightSpeed);
		}
		//		else if(WhiteController.getAButton() && !pidController.isEnabled() && !cameraController.isEnabled())
		//		{
		//			drive.driveToPosition(24);
		//		}

		//		if(WhiteController.getBButton())
		//		{
		//			drive.resetPosition();
		//		}
		SmartDashboard.putNumber("Ultrasonic", ultrasonic.getDistance());
		//		SmartDashboard.putBoolean("WhiteController A", WhiteController.getAButton());
		SmartDashboard.putBoolean("pidcontroller isenabled", pidController.isEnabled());
		SmartDashboard.putBoolean("cameracontroller isenabled", cameraController.isEnabled());
		SmartDashboard.putBoolean("Controller 6", Controller.getRawButton(6));
		SmartDashboard.putBoolean("Controller 5", Controller.getRawButton(5));
		SmartDashboard.putNumber("camera x", pixycamera.getVoltage());
		if(Controller.getPOV() == 90)
		{
			SmartDashboard.putNumber("cameraX Value", pixycamera.getVoltage());			//X value of Camera
			SmartDashboard.putNumber("SetPoint X", cameraController.getSetpoint());		// setpoint of cameraController
			SmartDashboard.putNumber("ERROR of Camera", cameraController.getError());   // Error (actual value - setpoint)
			SmartDashboard.putNumber("camera controller GET", cameraController.get());	// Speed output (-1000, 1000)
			SmartDashboard.putNumber("PixyCamera pidGet", pixycamera.pidGet());			// Camera output X value
			SmartDashboard.putNumber("loopValue", 4);
			cameraController.enable();
			cameraController.setSetpoint(0);
			drive.pidRotate();
		}else if(cameraController.isEnabled() && !(LaunchPad.getRawButton(7))){
			cameraController.disable();
		}

		if(Controller.getYButton())
		{
			SmartDashboard.putNumber("Right Master out", drive.getVoltage(false, true));
			SmartDashboard.putNumber("Left Master out", drive.getVoltage(true, true));
			SmartDashboard.putNumber("setpoint", pidController.getSetpoint());
			SmartDashboard.putNumber("getERROR", pidController.getError());
			pidController.enable();
			pidController.setInputRange(gyro.getAngle()-180, gyro.getAngle()+180);
			pidController.setSetpoint(turnTo(90));
			drive.pidRotate();
			double time2 = Timer.getFPGATimestamp();
			double error2 = pidController.getError();
			double errordot = Math.abs((error1 - error2)/(time1 - time2));
			time1 = time2;
			error1 = error2;
			SmartDashboard.putNumber("teleop time 1", time1);
			SmartDashboard.putNumber("teleop error 1", error1);
			SmartDashboard.putNumber("teleop time 2", time2);
			SmartDashboard.putNumber("teleop error 2", error2);
			SmartDashboard.putNumber("teleop errordot", errordot);
		}
		else if(pidController.isEnabled())
		{
			pidController.disable();
		}

		if(LaunchPad.getRawButton(7))
		{
			cameraController.enable();
			cameraController.setSetpoint(0);
			if(pixycamera.iscubeseen()){
				drive.driveFollow(ultrasonic.speed());
			}else
			{
				drive.pidRotate();
			}
		}
		else if(cameraController.isEnabled() && !(Controller.getPOV() == 90))
		{
			cameraController.disable();
		}
		//##########################################################################################################

		if(LaunchPad.getRawButton(12))
		{
			lift.setPosition(Parameters.ZERO_POSITION);
			setpoint = Parameters.ZERO_POSITION;
		}
		if(LaunchPad.getRawButton(6))
		{
			lift.setPosition(Parameters.SCALE_POSITION);
			setpoint = Parameters.SCALE_POSITION;
		}
		if(LaunchPad.getRawButton(8))
		{
			lift.setPosition(Parameters.SWITCH_POSITION);
			setpoint = Parameters.SWITCH_POSITION;
		}
		lift.setPosition(setpoint + (pots.getRawAxis(3))*(500));
		SmartDashboard.putNumber("LIFT POSIITON",lift.getPosition());

		//		if(lift.isLiftDown() && lift.getSetpoint() < lift.getPosition())
		//		{
		////			lift.resetPosition();
		//			lift.stopMotor();
		//		}
		//		if(!lift.isLiftUp() && lift.getSetpoint() > lift.getPosition())
		//		{
		//			lift.stopMotor();
		//		}
		//			SmartDashboard.putNumber("getERROR", pidController.getError());


		/*else {
			drive.set(leftSpeed, rightSpeed);
		}*/

		//		SmartDashboard.putBoolean("DigitalInput0", Dig1.get());
		//		SmartDashboard.putNumber("tankPressure", (((60/1.2)*(tank.getVoltage()))-15));
		//		SmartDashboard.putNumber("tank", tank.getVoltage());
		//	    double leftinch = Left_Drive_Master.getSelectedSensorPosition(0)/165.0;
		//		SmartDashboard.putNumber("Left Motor Position", (leftinch));
		//		double pressure = (Right_Drive_Follower.getSensorCollection().getAnalogIn()/4.23)-15;
		//		SmartDashboard.putNumber("Pressure", pressure);
		//		double rightinch = Right_Drive_Master.getSelectedSensorPosition(0)/165.0;
		//		SmartDashboard.putNumber("Right Motor Position", (rightinch));
		//		
		//
		//	
		//		// The teleopPeriodic method is entered each time the robot receives instructions it to be in teleoperated enabled mode
		//		SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
		//		if(Controller.getRawButton(5))
		//		{
		//			gyro.reset();
		//		}
		//		if(Controller.getRawButton(6))
		//		{
		//			gyro.calibrate();
		//		}
		//		
		//		boolean count = false;
		//		SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//		if(Controller.getRawButton(7))
		//		{
		//			SmartDashboard.putBoolean("CountBool", count);
		//			SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//			if(Controller.getRawButton(7) && count == false){
		//				SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//				Test_Talon.set(ControlMode.Position, testcount);
		//				testcount -= 1000;
		//				count = true;
		//			}
		//			while(count == true && Controller.getRawButton(7))
		//			{
		//				
		//			}
		//		}
		//		if(Controller.getPOV() == 270)
		//		{
		//			SmartDashboard.putBoolean("CountBool", count);
		//			SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//			if(Controller.getPOV() == 270 && count == false){
		//				SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//				Test_Talon.set(ControlMode.Position, 0);
		//				count = true;
		//			}
		//			while(count == true && Controller.getPOV() == 270)
		//			{
		//				
		//			}
		//		}
		//		if(Controller.getPOV() == 0)
		//		{
		//			SmartDashboard.putBoolean("CountBool", count);
		//			SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//			if(Controller.getPOV() == 0 && count == false){
		//				SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//				Test_Talon.set(ControlMode.Position, 6500);
		//				count = true;
		//			}
		//			while(count == true && Controller.getPOV() == 0)
		//			{
		//				
		//			}
		//		}
		//		if(Controller.getPOV() == 90)
		//		{
		//			SmartDashboard.putBoolean("CountBool", count);
		//			SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//			if(Controller.getPOV() == 90 && count == false){
		//				SmartDashboard.putNumber("QuadPosition", Test_Talon.getSensorCollection().getQuadraturePosition());
		//				Test_Talon.set(ControlMode.Position, 14500);
		//				count = true;
		//			}
		//			while(count == true && Controller.getPOV() == 90)
		//			{
		//				
		//			}
		//		}

		while(Controller.getRawButton(8))
		{
			if(Parameters.COMPRESSOR_AVAILABLE){
				comp.start();
			}
		}
		if(Parameters.COMPRESSOR_AVAILABLE){
			comp.stop();
		}

		boolean Trigger = false;
		if(Controller.getRawButton(9) && Trigger == false)
		{	
			drive.shiftGear();
			Trigger = true;
			while(Trigger == true && Controller.getRawButton(9))
			{

			}
		}

		boolean Trigger2 = false;
		if(Controller.getRawButton(10) && Trigger2 == false)
		{	
			drive.shiftPTO();
			Trigger2 = true;
			while(Trigger2 == true && Controller.getRawButton(10))
			{

			}
		}
		//		
		//		if(Controller.getRawButton(10))
		//		{
		//			pidDrive();
		//		}
		//		if(Controller.getPOV() == 225)
		//		{
		//			setAllFollower();
		//			SmartDashboard.putNumber("Left Trigger", Controller.getRawAxis(2));
		//		}
		//		if(Controller.getPOV() == 135)
		//		{
		//			resetFollowMode();
		//		}
		//		
		//		//Left_Drive_Master.set(2000);
		//		
		//
		////		SmartDashboard.putNumber("Auto_Selection", Auto_Selection);	
		//		
		////		SmartDashboard.putNumber("Left Master Speed", Left_Drive_Master.getSpeed());	
		////		SmartDashboard.putNumber("Left Master Position", Left_Drive_Master.getPosition());		
		//
		////		SmartDashboard.putNumber("Left Master Voltage", Left_Drive_Master.getOutputVoltage());	
		//		SmartDashboard.putNumber("Left Master Current", Left_Drive_Master.getOutputCurrent());	
		//		
		//		// =======================================================================================
		//		
		//		SmartDashboard.putNumber("Launch Pad Axis", LaunchPad.getX());
		//		SmartDashboard.putNumber("Launch Pad Y Axis", LaunchPad.getY());		
		//		
		//	      double Rotary_Switch_1_Value = LaunchPad.getX();
		//	      SmartDashboard.putNumber("Switch Raw Value", Rotary_Switch_1_Value);
		//		
		//		
		////		if (Math.abs(Controller.getRawAxis(5)) > .05)
		////		   {Left_Drive_Master.set(Controller.getRawAxis(5));
		//////		    Test_Talon.set(Controller.getRawAxis(5));
		////		   }
		////		else
		////		   {Left_Drive_Master.stopMotor();
		//////		    Test_Talon.stopMotor();
		////		   }
		//		
		//		if (Math.abs(Controller.getRawAxis(1)) > .05)
		//		   {drive.set(-Controller.getRawAxis(1));
		//		   
		//		   }
		//		else
		//		   {drive.set(0.0);
		//		   
		//		   }
		//
		//		
		//		double sonarvalue = sonar.getValue();
		//	    double sidesonarinches = ((sidesonar.getValue()-59.55)/7.727)+4;
		//		SmartDashboard.putNumber("Sonar_Value", sonarvalue);		
		//		SmartDashboard.putNumber("Camera X", camera_x.getAverageVoltage());
		//		SmartDashboard.putNumber("Camera_X", camera_x.getAverageVoltage());
		//		SmartDashboard.putNumber("Side Ultrasonic", sidesonar.getValue());
		//		SmartDashboard.putNumber("Side_Sonar_Inches", sidesonarinches);
		//		
		//		double sonarinches =(sonarvalue-59.55)/7.727;
		//		SmartDashboard.putNumber("Sonar_Inches", ((sonar.getValue()-59.55)/7.727));
		//	
		//		double test = SmartDashboard.getNumber("Value", 0);
		//		
		//		double speed = .25;
		//		
		//		while(Controller.getRawButton(6)) 
		//		{
		//			System.out.println("Hi");
		//		}
		//		while(Controller.getAButton( ) && (sonarinches > 12))
		//		{
		//			drive.set(speed);
		//			sonarvalue = sonar.getValue();
		//			sonarinches =(sonarvalue-59.55)/7.727;
		//			if (sonarinches < 6) speed = .10;
		//		}
		//
		//		while(Controller.getXButton( ) && (sonarinches > 24))
		//		{
		//			Right_Drive_Master.set(-speed);
		//			Left_Drive_Master.set(speed);
		//			sonarvalue = sonar.getValue();
		//			sonarinches =(sonarvalue-59.55)/7.727;
		//			if (sonarinches < 6) speed = .10;
		//		}
		//
		//		double gain = 0;
		//		
		//		while(Controller.getYButton( ))
		//		{
		//			//double inc = .05;
		//		    sidesonarinches = ((sidesonar.getValue()-59.55)/7.727)+4;
		//			speed = .25;
		//			Right_Drive_Master.set(speed + gain);
		//			Left_Drive_Master.set(-speed + gain);
		//			gain = 0;
		//			if (sidesonarinches < 11) gain = -.1;
		//			
		//			if (sidesonarinches > 13) gain = .1;
		//			//SmartDashboard.putNumber("speed", speed);
		//		}
		//		
		//
		//		
		//		while(Controller.getBButton( ) && (camera_x.getVoltage() < (1.3)))
		//			   {
		//			     Right_Drive_Master.set(-(speed+.05));
		//			     Left_Drive_Master.set(-(speed+.05));
		//			     
		//					SmartDashboard.putNumber("Camera X", camera_x.getAverageVoltage());	
		//			   }
		//		while(Controller.getBButton( ) && (camera_x.getVoltage() > (1.9)))
		//		   {
		//		     Right_Drive_Master.set(speed+.05);
		//		     Left_Drive_Master.set(speed+.05);
		//				SmartDashboard.putNumber("Camera X", camera_x.getAverageVoltage());		
		//		   }
		//		while(Controller.getBButton( ) && (camera_x.getVoltage() < (1.9) && (camera_x.getVoltage() > (1.3)))) 
		//		{
		//			while(Controller.getBButton() && ((sonar.getValue()-59.55)/7.727) > 7){
		//				double diff = (1.65 - camera_x.getVoltage())/2;
		//				double speeddiff = ((((sonar.getValue()-59.55)/7.727)-7)/250)+.2;
		//				Right_Drive_Master.set(-(speed+diff));
		//				Left_Drive_Master.set(speed);
		//				int floor =(int)((sonar.getValue()-59.55)/7.727)/10;
		//				
		//				switch(floor)
		//				{
		//				case 0:	speed = 0;
		//					break;
		//				case 1: speed = 0.15;
		//					break;
		//				case 2: speed = 0.2;
		//					break;
		//				case 3: speed = 0.3;
		//					break;
		//				case 4: speed = 0.4;
		//					break;
		//				case 5: speed = 0.45;
		//					break;
		//				case 6: speed = 0.5;
		//					break;
		//				case 7: speed = 0.6;
		//					break;
		//				default: speed = 0.7;
		//					break;
		//				}
		//				SmartDashboard.putNumber("Camera X", camera_x.getAverageVoltage());
		//				SmartDashboard.putNumber("Camera add Speed Difference", diff);
		//				SmartDashboard.putNumber("SpeedDifferenceByUltrasonic", speeddiff);
		////				if(camera_x.getVoltage() > 1.7 || camera_x.getVoltage() < 1.5)
		////				{
		////					break;
		////				}
		//			}
		//			Right_Drive_Master.stopMotor();
		//			Left_Drive_Master.stopMotor();
		//			SmartDashboard.putNumber("Camera X", camera_x.getAverageVoltage());	
		//		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		double Rotary_Switch_1_Value = LaunchPad.getX();
		SmartDashboard.putNumber("Switch Raw Value", Rotary_Switch_1_Value);
		SmartDashboard.putNumber("Tape Color", lineCamera.getColor());
		Scheduler.getInstance().run();
	}

	//	public void driveForward(double speed)
	//	{
	//		Right_Drive_Master.set(speed);
	//		Left_Drive_Master.set(-speed);
	//	}	
	//	
	//	public void driveBackward(double speed)
	//	{
	//		Right_Drive_Master.set(-speed);
	//		Left_Drive_Master.set(speed);	
	//	}	
	//	public void driveStop()
	//	{
	//		Right_Drive_Master.stopMotor();
	//		Left_Drive_Master.stopMotor();	
	//	}	
	//	
	//	public void driveTurnRight(double speed)
	//	{
	//        Right_Drive_Master.set(speed);
	//     
	//	    Left_Drive_Master.set(speed);
	//	    
	//		
	//	}	
	//	public void driveTurnLeft(double speed)
	//	{
	//        Right_Drive_Master.set(-speed);
	//	    Left_Drive_Master.set(-speed);
	//	}		
	//	
	//	
	//	public static void resetGyro()
	//	{
	//		gyro.reset();
	//	}
	//	/**
	//	 * This method takes an angle (in degrees) and rotates the robot to the given angle (positive
	//	 * angle rotates clockwise, negative angle rotates counter-clockwise)
	//	 * @param angle the amount of degrees the robot will rotate.
	//	 */
	//	public static void Rotate(int angle)
	//	{
	//		double speed = .27;
	//		gyro.reset();
	//			if(angle > 0)
	//			{
	//				while(gyro.getAngle() < angle-3){
	//				if(Math.abs(gyro.getAngle() - angle) < 20)
	//				{
	//					speed = .155;
	//				}
	//				Right_Drive_Master.set(speed);
	//				Left_Drive_Master.set(speed);
	//				}
	//			}
	//			else
	//			{	
	//				while(gyro.getAngle() > angle+3){
	//				if(Math.abs(gyro.getAngle() - angle) < 20)
	//				{
	//					speed = .155;
	//				}
	//				Right_Drive_Master.set(-speed);
	//				Left_Drive_Master.set(-speed);
	//				}
	//			}
	//		
	//	}
	//	
	//	static double pos = 0;
	//	public static void pidGyro(double angle)
	//	{
	//		setAllFollower();
	//		Right_Drive_Master.set(ControlMode.Position, pos);
	//		Right_Drive_Master.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
	//	}
	//	
	//	public static void pidDrive()
	//	{
	//		
	//		SmartDashboard.putNumber("MAG- Quad-Position", Test_Talon.getSensorCollection().getQuadraturePosition());		
	//		Test_Talon.set(ControlMode.Position, 10000);
	//		SmartDashboard.putNumber("MAG-angle", Test_Talon.getSelectedSensorPosition(0));	
	//	}
	//	/**
	//	 * This method takes a distance (in inches) and drives forward to the given distance
	//	 * @param distance This is the distance in inches the robot will drive to.
	//	 */
	//	public static void driveTo(double distance)
	//	{
	//		double speed = 0.2;
	//		while((Left_Drive_Master.getSelectedSensorPosition(0)/165.0) > -distance)
	//		{
	//			//SmartDashboard.putNumber("PositionAuto", (Left_Drive_Master.getSelectedSensorPosition(0)/165.0));
	//			Left_Drive_Master.set(-speed);
	//			Right_Drive_Master.set(speed);
	//		}
	//	}
	//	
	//	public static void driveStraight(double distance)
	//	{
	//		double rightinch = Right_Drive_Master.getSelectedSensorPosition(0)/165.0;
	//		double leftinch = Left_Drive_Master.getSelectedSensorPosition(0)/165.0;
	//		double difference = Math.abs(rightinch) - Math.abs(leftinch);
	//		double speed = 0.25;
	//	
	//		if(difference > 0.05)
	//		{
	//			SmartDashboard.putNumber("Differencestraight", difference);
	//			SmartDashboard.putNumber("Right Motor Position", (rightinch));
	//			SmartDashboard.putNumber("Left Motor Position", (leftinch));
	//			Right_Drive_Master.set(speed);
	//			Left_Drive_Master.set(-(speed + 0.125));
	//		}
	//	
	//		if(difference < -0.05)
	//		{
	//			SmartDashboard.putNumber("Differencestraight", difference);
	//			SmartDashboard.putNumber("Right Motor Position", (rightinch));
	//			SmartDashboard.putNumber("Left Motor Position", (leftinch));
	//			Right_Drive_Master.set((speed + 0.125));
	//			Left_Drive_Master.set(-speed);
	//		}
	//	
	//		if(difference < 0.05 && difference > -0.05 && Math.abs(leftinch) < distance)
	//		{
	//			SmartDashboard.putNumber("Differencestraight", difference);
	//			SmartDashboard.putNumber("Right Motor Position", (rightinch));
	//			SmartDashboard.putNumber("Left Motor Position", (leftinch));
	//			Right_Drive_Master.set(speed);
	//			Left_Drive_Master.set(-speed);
	//		}
	//	
	//		if(Math.abs(leftinch) > distance || Math.abs(rightinch) > distance)
	//		{
	//			Left_Drive_Master.stopMotor();
	//			Right_Drive_Master.stopMotor();
	//		}
	//	}
	//	
	//	public static void driveGyro(int distance)
	//	{
	//		double rightinch = Right_Drive_Master.getSelectedSensorPosition(0)/165.0;
	//		double leftinch = Left_Drive_Master.getSelectedSensorPosition(0)/165.0;
	//		double speed = 0.2;
	//		double angle = gyro.getAngle();
	//		double offset = .1;
	//		if(angle > 2)
	//		{
	//			Right_Drive_Master.set(speed);
	//			Left_Drive_Master.set(-(speed + offset));
	//		}
	//		if(angle < -2)
	//		{
	//			Right_Drive_Master.set(speed + offset);
	//			Left_Drive_Master.set(-speed);
	//		}
	//		if(angle > -2 && angle < 2  && Math.abs(leftinch) < distance)
	//		{
	//			Right_Drive_Master.set(speed);
	//			Left_Drive_Master.set(-speed);
	//		}
	//		if(Math.abs(rightinch) > distance || Math.abs(leftinch) > distance)
	//		{
	//			Right_Drive_Master.stopMotor();
	//			Left_Drive_Master.stopMotor();
	//		}
	//
	//	}
	//	
	//	public static void setAllFollower()
	//	{
	//		Right_Drive_Follower.set(ControlMode.Follower, 0);
	//		Right_Drive_Follower.follow(Right_Drive_Master);
	//		Left_Drive_Master.set(ControlMode.Follower, 0);
	//		Left_Drive_Master.follow(Right_Drive_Master);
	//		Left_Drive_Follower.set(ControlMode.Follower, 0);
	//		Left_Drive_Follower.follow(Right_Drive_Master);
	//	}
	//	
	//	public static void resetFollowMode()
	//	{
	//		Right_Drive_Follower.set(ControlMode.Follower, 0);
	//		Right_Drive_Follower.follow(Right_Drive_Master);
	//		Left_Drive_Master.set(ControlMode.PercentOutput, 0);
	//		Left_Drive_Master.setInverted(false);
	//		Left_Drive_Follower.set(ControlMode.Follower, 0);
	//		Left_Drive_Follower.follow(Left_Drive_Master);
	//		Left_Drive_Follower.setInverted(false);
	//	}

}  