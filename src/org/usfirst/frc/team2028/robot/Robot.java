package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
//import com.ctre.CANTalon;
//import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
	AutoTest4   autooption11;
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

		setpoint = Parameters.LIFT_ZERO_POSITION;


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

	//============================================AUTONOMOUS==============AUTONOMOUS=======================
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
		autooption3 = new AutoOption3(pidController, drive, lift, gripper, ultrasonic, delay, positionknob,
				leftswitch, gamedata);
		autooption4 = new AutoOption4(pidController, drive, lift, gripper, ultrasonic, lineCamera, delay, positionknob,
				leftscale, gamedata);
		autooption5 = new AutoOption5(pidController, drive, lift, gripper, ultrasonic, lineCamera, delay, positionknob,
				leftswitch, gamedata);
		autooption6 = new AutoOption6(pidController, drive, lift, gripper, ultrasonic, lineCamera, delay, positionknob,
				leftscale, gamedata);
		autooption11 = new AutoTest4(pidController,  drive,  lift, ultrasonic,
				 positionknob,  leftswitch, gamedata);

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
		case 11:
			autooption11.start();
		default:
			break;
		}
		SmartDashboard.putBoolean("autostart", true);

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

	}
	//#################################TELEOP##########################TELEOP##################################
	double time1;
	double error1;
	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
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
		double airpressure = tank.getAverageVoltage();
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
		//		if(Controller.getPOV() == 270)
		//		{
		//			inc += 50;
		//			lift.setPosition(inc);
		//		}
		//		if(Controller.getPOV() == 315)
		//		{
		//			inc -= 50;
		//			lift.setPosition(inc);
		//		}
		//		if(Controller.getPOV() == 225)
		//		{
		//			lift.zeroPosition();
		//		}

		Controller.setRumble(RumbleType.kLeftRumble, 0);
		Controller.setRumble(RumbleType.kRightRumble, 0);
		if(LaunchPad.getRawButton(5))
		{
			if(gripper.isCubeHeld())
			{
				Controller.setRumble(RumbleType.kLeftRumble, 1);
				Controller.setRumble(RumbleType.kRightRumble, 1);
			}
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

		boolean Trigger2 = false;
		if(LaunchPad.getRawButton(10))
		{
			ramp.retractOutrigger();

			if(Controller.getRawButton(10) && Trigger2 == false)
			{	
				drive.shiftPTO();
				Trigger2 = true;
				while(Trigger2 == true && Controller.getRawButton(10))
				{
	
				}
			}

		}
		if(LaunchPad.getRawButton(11))
		{
			ramp.deployOutriggers();
		}

		if((Controller.getRawButton(6) && !pidController.isEnabled()) || 
				(Controller.getRawButton(6) && !cameraController.isEnabled()))
		{
			SmartDashboard.putNumber("loopValue", 1);
			drive.rotate(leftStickSpeed);
		} 
		else if ((Controller.getRawButton(5) && !pidController.isEnabled()) || 
				(Controller.getRawButton(5) && !cameraController.isEnabled())) 
		{
			SmartDashboard.putNumber("loopValue", 2);
			drive.set(leftStickSpeed);
		}
		else if(/*!WhiteController.getAButton() && */!pidController.isEnabled() && !cameraController.isEnabled())
		{
			SmartDashboard.putNumber("loopValue", 3);
			drive.setVoltage(leftSpeed, rightSpeed);
		}

		SmartDashboard.putNumber("Ultrasonic", ultrasonic.getDistance());
		//		SmartDashboard.putBoolean("WhiteController A", WhiteController.getAButton());
		SmartDashboard.putBoolean("pidcontroller isenabled", pidController.isEnabled());
		SmartDashboard.putBoolean("cameracontroller isenabled", cameraController.isEnabled());
		SmartDashboard.putBoolean("Controller 6", Controller.getRawButton(6));
		SmartDashboard.putBoolean("Controller 5", Controller.getRawButton(5));
		SmartDashboard.putNumber("camera x", pixycamera.getVoltage());
		if(Controller.getBButton())
		{
			Controller.setRumble(RumbleType.kLeftRumble, 1);
			Controller.setRumble(RumbleType.kRightRumble, 1);
		}
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
			lift.setPosition(Parameters.LIFT_ZERO_POSITION);
			setpoint = Parameters.LIFT_ZERO_POSITION;
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

//		boolean Trigger2 = false;
//		if(Controller.getRawButton(10) && Trigger2 == false)
//		{	
//			drive.shiftPTO();
//			Trigger2 = true;
//			while(Trigger2 == true && Controller.getRawButton(10))
//			{
//
//			}
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
		if(Controller.getXButton())
		{
			drive.set(200);
		}
		drive.set(0);
		Scheduler.getInstance().run();
	}


}  