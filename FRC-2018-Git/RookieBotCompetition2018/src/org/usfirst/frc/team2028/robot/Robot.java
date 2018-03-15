package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
//import com.ctre.CANTalon;
//import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import org.usfirst.frc.team2028.robot.Parameters.TapeColor;

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
import edu.wpi.first.wpilibj.command.Command;

//import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Robot extends IterativeRobot {
	//	 static ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);
	LineCamera linecamera;
	static Gyro gyro = new Gyro();
	PIDController pidController;
	PIDController cameraController;
	XboxController Controller = new XboxController(0);		//FIXME make C lowercase
	//	 XboxController WhiteController = new XboxController(4); 
	Timer timer = new Timer();
	PixyCamera pixycamera = new PixyCamera();
	Joystick LaunchPad = new Joystick(Parameters.CONTROLBOARD_BUTTONS_PORT);
	Joystick pots = new Joystick(Parameters.CONTROLBOARD_POTS_PORT);
	Joystick switches = new Joystick(Parameters.CONTROLBOARD_SWITCHES_PORT);
	Ultrasonic frontultrasonic = new Ultrasonic(Parameters.ULTRASONIC_FRONT_ANALOG_PORT);
	Ultrasonic rearultrasonic = new Ultrasonic(Parameters.ULTRASONIC_REAR_ANALOG_PORT);
	Compressor comp;
	DeployRampCommand deployramp;
	LiftRampCommand liftramp;
	DriveToPositionAuto drivetopos;
	//	DriveAutoLineCommand autobaseline;
	AutoOption1 autooption1;
	AutoOption2 autooption2;
	AutoOption3 autooption3;
	AutoOption4 autooption4;
	AutoOption5 autooption5;
	AutoOption6 autooption6;
	AutoOption7 autooption7;
	AutoTest4   autooption11;
	Command joystickdrive = null;
	Command testFunc;


	AnalogInput sonar;
	//	 AnalogInput camera_x;
	AnalogInput tank;

	Timer myTimer;

	private double delay;
	private int positionknob;
	private int objectiveknob;
	private int delayknob;
	private double setpoint;
	private double inc;
	private Ramp ramp;
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
	private boolean endgame = false;
	private double rightSpeed;
	private double leftSpeed;
	private double turnSpeed;
	private double timebefore;
	// 	 

	public Robot() {
		super();
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("Initializing Robot");
		linecamera = new LineCamera();
		ramp = new Ramp();
		lift = new Lift();
		drive = new Drive(this, Controller, rearultrasonic);
		gripper = new Gripper();
		pidController = new PIDController(Parameters.Pid.CONTROLLER.getP() , Parameters.Pid.CONTROLLER.getI() 
				, Parameters.Pid.CONTROLLER.getD(), Parameters.Pid.CONTROLLER.getF() , gyro , drive);
		pidController.setOutputRange(-1000, 1000);
		cameraController = new PIDController(Parameters.Pid.CAMERA.getP(), Parameters.Pid.CAMERA.getI(), Parameters.Pid.CAMERA.getD(), 
				Parameters.Pid.CAMERA.getF(), pixycamera, drive);
		cameraController.setOutputRange(-3000, 3000);
		cameraController.setInputRange(-1.65, 1.65);
		drivetopos = new DriveToPositionAuto(pidController, drive, 24.0, 20 , rearultrasonic);
		deployramp = new DeployRampCommand(drive);
		liftramp = new LiftRampCommand(drive);
		//		 gyro.reset();



		drive.setPTOHigh();

		setpoint = Parameters.LIFT_ZERO_POSITION;


		//	 	 sonar    = new AnalogInput(0);
		//		 camera_x = new AnalogInput(1);

		tank = new AnalogInput(3);
		if(Parameters.COMPRESSOR_AVAILABLE){
			comp = new Compressor();
		}
		myTimer = new Timer();

		joystickdrive = drive.getDefCommand();
		System.out.println("JoystickDrive command = " + (joystickdrive==null?"NULL":joystickdrive.toString()));

		Scheduler.getInstance().enable();
		System.out.println("done initializing robot.");
	}


	public int decodeRotary(int axisnumber)
	{
		double[] Rotary_Switch_1 = { 
				-1.000,       //  1 
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

		//		if(Math.random() < 2)
		//			return;
		positionknob = decodeRotary(0);
		objectiveknob = decodeRotary(1);
		delayknob = decodeRotary(2);

		double[] speed = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500};
		TapeColor greenswitch;
		double Rotary_Switch_1_Value = LaunchPad.getX();
		SmartDashboard.putNumber("Switch Raw Value", Rotary_Switch_1_Value);
		if (Parameters.LINE_CAMERA_AVAILABLE) {
			TapeColor colorSeen = linecamera.getColor();
			SmartDashboard.putString("Tape Color", colorSeen.toString());
		}

		if(switches.getRawButton(Parameters.GREEN_SWITCH))
		{
			greenswitch = Parameters.TapeColor.BLACK;
		}else
		{
			greenswitch = Parameters.TapeColor.WHITE;
		}

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
		SmartDashboard.putNumber("knob", objectiveknob);

		testFunc = new TestGroup(objectiveknob, drive, positionknob, delayknob,
				greenswitch, lift, gripper, gyro, linecamera,
				pots.getRawAxis(Parameters.GRIPPER_TILT_POT));

		//		autobaseline = new DriveAutoLineCommand(drive, delay);
		autooption1 = new AutoOption1(pidController, drive, delay, positionknob, leftswitch);

		autooption2 = new AutoOption2(pidController, drive, rearultrasonic, delay, positionknob,
				leftswitch, gamedata);
		autooption3 = new AutoOption3(pidController, drive, lift, gripper, rearultrasonic, delay, positionknob,
				leftswitch, gamedata);
		autooption4 = new AutoOption4(pidController, drive, lift, gripper, rearultrasonic, lineCamera, delay, positionknob,
				leftscale, gamedata);
		autooption5 = new AutoOption5(pidController, drive, lift, gripper, rearultrasonic, lineCamera, delay, positionknob,
				leftswitch, gamedata);
		autooption6 = new AutoOption6(pidController, drive, lift, gripper, rearultrasonic, lineCamera, delay, positionknob,
				leftscale, gamedata);
		autooption11 = new AutoTest4(pidController,  drive,  lift, rearultrasonic,
				positionknob,  leftswitch, gamedata);

		if(Parameters.TEST_MODE)
		{
			testFunc.start();
		}
		else
		{
			switch(objectiveknob)
			{
			//			case 1:
			//				autobaseline.start();
			//				break;
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
				break;
			default:
				break;
			}
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

	private void auto1(double timeRemaining)
	{
		if(timeRemaining > 12)
		{
			drive.setVoltage(0.43, 0.43);
		}
		else
		{
			drive.stop();
		}
	}

	private void auto2(double timeRemaining)
	{
		if(rearultrasonic.getDistance() < 65 && timeRemaining >= 10.0)
		{
			SmartDashboard.putNumber("ultrasonic value auto", rearultrasonic.getDistance());
			drive.setVoltage(0.35, 0.35);
		}
		else 
		{
			drive.stop();
			if(timeRemaining >= 8.5 && timeRemaining < 10.0)
			{
				drive.stop();
				lift.setPosition(Parameters.LIFT_SCALE_POSITION + 1000);
			}
			else if(timeRemaining >= 6.5 && timeRemaining < 8.5)
			{
				lift.stopMotor();
				drive.setVoltage(0.2, 0.2);
			}
			else if(timeRemaining >= 4.0 && timeRemaining < 6.5)
			{
				drive.stop();
				lift.stopMotor();
				gripper.ejectCube();
			}
			else
			{
				lift.stopMotor();
				gripper.stopGripper();
			}

		}


	}
	
	private void auto3(double timeRemaining, String gamedata)
	{
		if(timeRemaining >= 12.0)
		{
			drive.setVoltage(0.6, 0.6); // Drives forward for 3 seconds
			drive.resetRotatePosition();
		}
		else if(!switches.getRawButton(Parameters.BLUE_SWITCH) && gamedata.charAt(0) == 'L')
		{
			if(timeRemaining >= 11.0 && timeRemaining < 12.0)
			{
				drive.rotateToPosition(-90, 0.5);//rotates 90 degrees to the right
			}
			else if(timeRemaining >= 9.5 && timeRemaining < 11.0)
			{
				drive.stop(); //stops drive motors
				if(Parameters.LIFT_AVAILABLE){
					//lifts the lift motor for 1.5 seconds
					lift.setPosition(Parameters.LIFT_SCALE_POSITION + 1000); 
				}
			}
			else if(timeRemaining >= 8.5 && timeRemaining < 9.5)
			{
				if(Parameters.LIFT_AVAILABLE)
				{
					lift.stopMotor();
				}
				//drives forward to inch closer to the switch.
				drive.setVoltage(0.4, 0.4);
			}
			else if(timeRemaining >= 7.0 && timeRemaining < 8.5)
			{
				drive.stop();//stops inching closer
				if(Parameters.GRIPPER_AVAILABLE){
					//ejects a cube onto the switch
					gripper.ejectCube();
				}
			}
			else
			{
				//stop everything just in case
				if(Parameters.GRIPPER_AVAILABLE){
					gripper.stopGripper();
				}
				if(Parameters.LIFT_AVAILABLE){
					lift.stopMotor();
				}
				drive.stop();
				drive.resetRotatePosition();
			}
		} // vvv checks if our switch aligns with where we are placed on the field vvv
		else if(switches.getRawButton(Parameters.BLUE_SWITCH) && gamedata.charAt(0) == 'R')
		{
			if(timeRemaining >= 11.0 && timeRemaining < 12.0)
			{
				drive.rotateToPosition(90, 0.5); // rotates 90 degrees to the right
			}
			else if(timeRemaining >= 9.5 && timeRemaining < 11.0)
			{
				drive.stop(); // stops rotating
				if(Parameters.LIFT_AVAILABLE){
					//lifts the motor to the switch position
					lift.setPosition(Parameters.LIFT_SCALE_POSITION + 1000);
				}
			}
			else if(timeRemaining >= 8.5 && timeRemaining < 9.5)
			{
				if(Parameters.LIFT_AVAILABLE)
				{
					lift.stopMotor();
				}
				drive.setVoltage(0.4, 0.4); // inches closer to the switch
			}
			else if(timeRemaining >= 7.0 && timeRemaining < 8.5)
			{
				drive.stop(); // stops driving
				if(Parameters.GRIPPER_AVAILABLE){
					gripper.ejectCube(); // ejects a cube
				}
			}
			else
			{
				if(Parameters.GRIPPER_AVAILABLE){
					gripper.stopGripper();
				}
				if(Parameters.LIFT_AVAILABLE){
					lift.stopMotor();
				}
				drive.stop();
				drive.resetRotatePosition();
			}
		}
		else
		{
			auto1(timeRemaining);
		}
	}	

	public void auto4(double timeRemaining, String gamedata)
	{
		if(timeRemaining >= 7.0){
			auto3(timeRemaining, gamedata);
		}
		else if(!switches.getRawButton(Parameters.BLUE_SWITCH) && gamedata.charAt(0) == 'L')
		{//left side of switch and auto3 worked
			if(timeRemaining >= 6.5 && timeRemaining < 7.0)
			{
				drive.setVoltage(-0.2, -0.2);
				drive.resetRotatePosition();
			}
			else if(timeRemaining >= 5.5 && timeRemaining < 6.5)
			{
				drive.rotateToPosition(-90, 0.5);
			}
			else if(timeRemaining >= 5.0 && timeRemaining < 5.5)
			{
				drive.setVoltage(0.2, 0.2);
				drive.resetRotatePosition();
			}
			else if(timeRemaining >= 4.0 && timeRemaining < 5.0)
			{
				drive.rotateToPosition(90, 0.50);
				lift.setPosition(Parameters.LIFT_FLOOR_POSITION);
			}
			else
			{
				drive.stop();
				lift.stopMotor();
			}
		}
		else if(switches.getRawButton(Parameters.BLUE_SWITCH) && gamedata.charAt(1) == 'R')
		{//right side of switch and auto3 worked
			if(timeRemaining >= 6.5 && timeRemaining < 7.0)
			{
				drive.setVoltage(-0.2, -0.2);
				drive.resetRotatePosition();
			}
			else if(timeRemaining >= 5.5 && timeRemaining < 6.5)
			{
				drive.rotateToPosition(90, 0.5);
			}
			else if(timeRemaining >= 5.0 && timeRemaining < 5.5)
			{
				drive.setVoltage(0.2, 0.2);
				drive.resetRotatePosition();
			}
			else if(timeRemaining >= 4.0 && timeRemaining < 5.0)
			{
				drive.rotateToPosition(-90, 0.5);
				lift.setPosition(Parameters.LIFT_FLOOR_POSITION);
			}
			else
			{
				drive.stop();
				lift.stopMotor();
			}
		}
		else
		{
			drive.stop();
		}

	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Gyro value", gyro.pidGet());
		String gamedata = DriverStation.getInstance().getGameSpecificMessage();
		gamedata = gamedata.toUpperCase();
		double timeRemaining = Timer.getMatchTime();
		System.out.println(timeRemaining);
		drive.setLowGear();
		switch(objectiveknob)
		{
		case 1:
			auto1(timeRemaining);
			break;
		case 2:
			if(!switches.getRawButton(Parameters.BLUE_SWITCH) && gamedata.charAt(0) == 'L')
			{
				auto2(timeRemaining);
			}
			else if (switches.getRawButton(Parameters.BLUE_SWITCH) && gamedata.charAt(0) == 'R')
			{
				auto2(timeRemaining);
			}
			else
			{
				auto1(timeRemaining);
				gripper.stopGripper();
			}
			break;
		case 3:
			auto3(timeRemaining, gamedata);
			break;
		case 4:
			auto4(timeRemaining, gamedata);
			break;
		case 10:
			SmartDashboard.putNumber("current auto time", timeRemaining);
			drive.rotateToPosition(90, 0.5);
			if(Math.abs(drive.getRightMotorOutputPercent()) < 0.05 ||
					Math.abs(drive.getLeftMotorOutputPercent()) < 0.05)
			{
				SmartDashboard.putNumber("time for drive rotate to complete", timeRemaining);
			}
			break;
		default:
			drive.setVoltage(0.0, 0.0);
			gripper.stopGripper();
			lift.stopMotor();
			break;
		}
		drive.go();
		//		if(Parameters.COMPRESSOR_AVAILABLE){
		//			comp.stop();
		//		}
		//		Scheduler.getInstance().run();
		//	
		//		//		SmartDashboard.putBoolean("can you see me", true);
		//		//		SmartDashboard.putBoolean("leftswitch", leftswitch);
		//		//		DriveToPositionAuto(drive, 10);

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
		timebefore = timer.getFPGATimestamp();
		testcount = 0;
		drive.resetPosition();
		time1 = Timer.getFPGATimestamp();
		error1 = pidController.getError();
		drive.setPTOHigh();
		endgame = false;

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

		//		double pressure = (250*(tank.getVoltage()/12)) - 25;
		//		SmartDashboard.putNumber("tank pressure", pressure);

		//		Scheduler.getInstance().run();



		SmartDashboard.putNumber("Lift_Reading", lift.getPosition());		
		SmartDashboard.putNumber("Lift setpoint", lift.getSetpoint());
		SmartDashboard.putBoolean("is lift down", lift.isLiftDown());
		SmartDashboard.putBoolean("is lift up", lift.isLiftUp());
		SmartDashboard.putNumber("lift voltage", lift.getVoltage());
		SmartDashboard.putBoolean("is at setpont LIFT", lift.isAtSetPoint());

		drive.go();
		if(Parameters.COMPRESSOR_AVAILABLE)
		{
			comp.stop();
		}


		double time = timer.getFPGATimestamp();
		double rightStickSpeed = Controller.getRawAxis(1) * -4069 * 500 / 600;
		double leftStickSpeed  = Controller.getRawAxis(5) * -4069 * 500 / 600;



		drive.go();

		if (Math.abs(Controller.getRawAxis(1)) > .1)
		{
			rightSpeed = -(Controller.getRawAxis(1));
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
		if (Math.abs(Controller.getRawAxis(4)) > .1)
		{
			turnSpeed = Controller.getRawAxis(4);
		}
		else
		{
			turnSpeed = 0;
		}

		if((Controller.getRawButton(6) && !isTurning()) || 
				(Controller.getRawButton(6) && !isFollowingCube()))
		{
			SmartDashboard.putNumber("loopValue", 1);
			drive.rotate(leftStickSpeed);
		} 
		else if ((Controller.getRawButton(5) && !isTurning()) || 
				(Controller.getRawButton(5) && !isFollowingCube())) 
		{
			SmartDashboard.putNumber("loopValue", 2);
			SmartDashboard.putNumber("left side velocity", -drive.getRightVelocity());
			SmartDashboard.putNumber("right side velocity", -drive.getLeftVelocity());
			//			drive.resetPosition();
			if(Controller.getRawButtonPressed(5))
			{
				drive.resetDriveToPositions();
			}
			drive.driveReverse(160);
			//			drive.set(50);
		}
		else if(!isTurning() && !isFollowingCube())
		{
			SmartDashboard.putNumber("loopValue", 3);
			//			if(Parameters.VIDEO_GAME_DRIVE_MODE)
			//			{
			//				drive.gameDrive(leftSpeed, turnSpeed);
			//			}
			//			else{
			//			double[] smooths = smooth(leftSpeed, rightSpeed, time);
			//			drive.setVoltage(smooths[0], smooths[1]);
			drive.setVoltage(leftSpeed, rightSpeed);
			//			}			
		}
		else if(Controller.getRawAxis(3) > 0.5 && !isFollowingCube() ||
				Controller.getRawAxis(3) > 0.5 && !isTurning())
		{
			drive.set(rearultrasonic.speed());
		}


		double airpressure = tank.getAverageVoltage();
		if(Parameters.LIFT_AVAILABLE){
			SmartDashboard.putBoolean("FwdLimitSwitch", lift.isLiftUp());
			SmartDashboard.putBoolean("RevLimitSwitch", lift.isLiftDown());
			SmartDashboard.putNumber("P VALUE", Parameters.Pid.CAMERA.getP());
			SmartDashboard.putNumber("Right Master out", drive.getVoltage(false, true));
			SmartDashboard.putNumber("Left Master out", drive.getVoltage(true, true));
			SmartDashboard.putNumber("lift position", lift.getPosition());
		}
		SmartDashboard.putNumber("right position", drive.getrightposition());
		SmartDashboard.putNumber("left position", drive.getleftposition());

		SmartDashboard.putNumber("max motor current", drive.getMaximumMotorCurrent());
		//		if(LaunchPad.getRawButton(9) && switches.getRawButton(Parameters.RED_SWITCH))
		//		{
		//			//			decodeRotary(0);
		//			//			decodeRotary(1);
		//			//			decodeRotary(2);
		//			//			SmartDashboard.putNumber("adjust pot", pots.getRawAxis(3));
		//			//			SmartDashboard.putNumber("Gripper pot", pots.getRawAxis(4));
		//			drive.set(275);
		//		}
		//		else
		//		{
		//			drive.set(0);
		//		}

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

		SmartDashboard.putNumber("Gyro value", gyro.pidGet());
		Controller.setRumble(RumbleType.kLeftRumble, 0);
		Controller.setRumble(RumbleType.kRightRumble, 0);


		if(LaunchPad.getRawButton(5))
		{
			//			if(gripper.isCubeHeld())
			//			{
			//				Controller.setRumble(RumbleType.kLeftRumble, 1);
			//				Controller.setRumble(RumbleType.kRightRumble, 1);
			//			}
			gripper.pickupCube();
		}
		else if(LaunchPad.getRawButton(1))
		{
			gripper.dribbleCube();
		}
		else if(LaunchPad.getRawButton(2))
		{
			gripper.ejectCube();
		}
		else if(LaunchPad.getRawButton(3))
		{
			gripper.launchCube();
		}
		else if(gripper.isOn())
		{
			gripper.stopGripper();
		}




		SmartDashboard.putString("PTOstate",drive.getPTOHigh().toString());
		if(LaunchPad.getRawButton(10) && endgame && switches.getRawButton(Parameters.RED_SWITCH))
		{
			SmartDashboard.putString("PTOstate",drive.getPTOHigh().toString());
			System.out.println("lower outrigger button 10 pressed");
			ramp.deployRamps();
		}
		ramp.openGeorge();
		if(LaunchPad.getRawButton(11)
				/*&& !liftramp.isStarted() */ && switches.getRawButton(Parameters.RED_SWITCH))
		{
			//			liftramp.start();
			drive.setLowGear();
			drive.setPTOLow();
			drive.set(-500);
			endgame = true;
		}
		else if(switches.getRawButton(Parameters.RED_SWITCH) && endgame)
		{
			drive.stop();
		}



		if(endgame)
		{
			drive.setDefaultCommand(null);
		}

		SmartDashboard.putNumber("RearUltrasonic", rearultrasonic.getDistance());
		SmartDashboard.putNumber("FrontUltrasonic", frontultrasonic.getDistance());
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
		}
		else if(cameraController.isEnabled() && !(LaunchPad.getRawButton(7)))
		{
			cameraController.disable();
		}

		if(!Parameters.COMPRESSOR_AVAILABLE){
			if(Controller.getStartButton())
			{
				lift.zeroPosition();
			}
		}

		if(Parameters.TEST_MODE_GRIPPER){
			if(Controller.getYButton() || pots.getRawAxis(Parameters.GRIPPER_TILT_POT) > 0.3)
			{
				gripper.setVoltage(0.5);
				//			SmartDashboard.putNumber("Right Master out", drive.getVoltage(false, true));
				//			SmartDashboard.putNumber("Left Master out", drive.getVoltage(true, true));
				//			SmartDashboard.putNumber("setpoint", pidController.getSetpoint());
				//			SmartDashboard.putNumber("getERROR", pidController.getError());
				//			pidController.enable();
				//			pidController.setInputRange(gyro.getAngle()-180, gyro.getAngle()+180);
				//			pidController.setSetpoint(turnTo(90));
				//			drive.pidRotate();
				//			double time2 = Timer.getFPGATimestamp();
				//			double error2 = pidController.getError();
				//			double errordot = Math.abs((error1 - error2)/(time1 - time2));
				//			time1 = time2;
				//			error1 = error2;
				//			SmartDashboard.putNumber("teleop time 1", time1);
				//			SmartDashboard.putNumber("teleop error 1", error1);
				//			SmartDashboard.putNumber("teleop time 2", time2);
				//			SmartDashboard.putNumber("teleop error 2", error2);
				//			SmartDashboard.putNumber("teleop errordot", errordot);
			}
			else if(Controller.getAButton() || pots.getRawAxis(Parameters.GRIPPER_TILT_POT) < -0.3)
			{
				gripper.setVoltage(-0.2);
			}
			else
			{
				gripper.stopTilt();
			}
		}
		if(LaunchPad.getRawButton(7))
		{
			SmartDashboard.putNumber("camera controller GET", cameraController.get());	// Speed output (-1000, 1000)
			SmartDashboard.putNumber("PixyCamera pidGet", pixycamera.pidGet());		
			//			followCubeCommand = new FollowCubeCommand(...);
			//			followCubeCommand.start();
			cameraController.enable();
			cameraController.setSetpoint(0);
			if(pixycamera.iscubeseen()){
				drive.driveFollow(frontultrasonic.speed());
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
			//			lift.setPosition(Parameters.LIFT_ZERO_POSITION);
			setpoint = Parameters.LIFT_ZERO_POSITION;
			lift.setPosition(setpoint + (pots.getRawAxis(3))*(-2000));
		}
		else if(LaunchPad.getRawButton(6))
		{
			//			lift.setPosition(Parameters.LIFT_SCALE_POSITION);
			setpoint = Parameters.LIFT_SCALE_POSITION;
			lift.setPosition(setpoint + (pots.getRawAxis(3))*(-2000));
		}
		else if(LaunchPad.getRawButton(8))
		{
			//			lift.setPosition(Parameters.LIFT_SWITCH_POSITION);
			setpoint = Parameters.LIFT_SWITCH_POSITION;
			lift.setPosition(setpoint + (pots.getRawAxis(3))*(-2000));
		}
		else if(Controller.getPOV() == 0)
		{
			lift.setVoltage(0.6);
		}
		else if (Controller.getPOV() == 180)
		{
			lift.setVoltage(-0.6);
		}
		else
		{
			lift.stopMotor();
		}


		SmartDashboard.putNumber("tilt motor position", gripper.getTiltPosition());
		if(Controller.getRawButton(7))
		{
			gripper.resetTiltPosition();
		}
		//		if(pots.getRawAxis(4) > -0.97 && !Parameters.TEST_MODE){
		//			gripper.tiltTo(((pots.getRawAxis(4)+1))*(50));
		//		}
		//		else if (!Parameters.TEST_MODE)
		//		{
		//			gripper.stopTilt();
		//		}
		//		lift.setPosition(setpoint + (pots.getRawAxis(3))*(-2000));

		if(Parameters.LIFT_AVAILABLE){	
			SmartDashboard.putNumber("pto_current", drive.getMaximumMotorCurrent());
			SmartDashboard.putNumber("LIFT POSITION",lift.getPosition());
		}
		if(Parameters.COMPRESSOR_AVAILABLE){
			if(Controller.getRawButton(8))
			{
				comp.start();
			}
			else {
				comp.stop();
			}
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



	public boolean isTurning()
	{
		return pidController.isEnabled();
	}

	public boolean isFollowingCube()
	{
		return cameraController.isEnabled();
	}

	@Override
	public void testInit() { 

		positionknob = decodeRotary(0); // use it for running the desired test
		objectiveknob = decodeRotary(1); // use it for determining test parameters
		delayknob = decodeRotary(2); //use it for determining more test parameters (if needed)
		// Do nothing
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		drive.setVoltage(0.5, 0.5);
		drive.go();
		//		double[] speed = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500};
		//		TapeColor greenswitch;
		//		double Rotary_Switch_1_Value = LaunchPad.getX();
		//		SmartDashboard.putNumber("Switch Raw Value", Rotary_Switch_1_Value);
		//		if (Parameters.LINE_CAMERA_AVAILABLE) {
		//			TapeColor colorSeen = lineCamera.getColor();
		//			SmartDashboard.putString("Tape Color", colorSeen.toString());
		//		}
		//		//		boolean greenswitch = switches.getRawButton(0);
		//		if(switches.getRawButton(Parameters.GREEN_SWITCH))
		//		{
		//			greenswitch = Parameters.TapeColor.BLACK;
		//		}else
		//		{
		//			greenswitch = Parameters.TapeColor.WHITE;
		//		}
		//		// Run the desired test
		//
		//		switch(decodeRotary(Parameters.POSITION_KNOB))		
		//		{
		//		case 1:
		//			testFunc = new LineTest(drive, lineCamera, greenswitch, decodeRotary(Parameters.OBJECTIVE_KNOB));
		//			break;
		//		case 2:
		//			testFunc = new TestRotate(0, 0, drive, decodeRotary(Parameters.OBJECTIVE_KNOB),
		//					decodeRotary(Parameters.DELAY_KNOB), gyro);
		//			break;
		//		case 3:
		//			testFunc = new TestExpel(gripper, decodeRotary(Parameters.OBJECTIVE_KNOB));
		//			break;
		//		case 4:
		//			testFunc = new TestAngleMotor(gripper, decodeRotary(Parameters.OBJECTIVE_KNOB), decodeRotary(Parameters.DELAY_KNOB),
		//					pots.getRawAxis(Parameters.GRIPPER_TILT_POT));
		//		}		
		//		testFunc.start();
		//		Scheduler.getInstance().run();
	}


}  