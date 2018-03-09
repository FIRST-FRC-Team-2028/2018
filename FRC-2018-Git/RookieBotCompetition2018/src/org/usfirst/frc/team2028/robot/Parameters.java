package org.usfirst.frc.team2028.robot;

public class Parameters {

	/**
	 * Flags to control condition hardware availability between competition and rookie/practice bot
	 */
	public static final boolean LINE_CAMERA_AVAILABLE = false;
	public static final boolean COMPRESSOR_AVAILABLE = false;
	public static final boolean RAMP_AVAILABLE = true;
	public static final boolean GRIPPER_AVAILABLE = true;
	public static final boolean LIFT_AVAILABLE = true;
	public static final boolean GYRO_AVAILABLE = true;
	public static final boolean TEST_MODE = true;
	public static final boolean VIDEO_GAME_DRIVE_MODE = false;
	
	public static final int ULTRASONIC_FRONT_ANALOG_PORT = 1;
	public static final int ULTRASONIC_REAR_ANALOG_PORT = 0;
	public static final int PIXY_CAMERA_CUBE_PORT = 2;
	
	public static final int CONTROLBOARD_BUTTONS_PORT = 1;
	public static final int CONTROLBOARD_POTS_PORT = 2;
	public static final int CONTROLBOARD_SWITCHES_PORT = 3;
	// basic robot geometry
	public static final double ROBOT_LENGTH = 32.75;
	
	public static final boolean RIGHT_PHASE = false;
	public static final boolean LEFT_PHASE = false;
	public static final boolean LEFT_DRIVE_INVERTED = false;
	public static final boolean LEFT_GRIPPER_INVERTED = true;
	
	public enum Pid {
		
		LEFT_DRIVE_MOTOR(0.3, 0.001, 0.00, 0.0),	// TODO: CHANGE PID VALUES FOR DRIVE FIXME: CHANGE PID VALUES FOR DRIVE
		RIGHT_DRIVE_MOTOR(0.3, 0.001, 0.00, 0.0),
//		CONTROLLER(9, 0.008, 0.005, 0.0)
		CONTROLLER(3.23, 0.012, 0.23, 0),//D was 0.5
		CAMERA(18.0, 0.1, 0.007, 0.0),
		LIFT(10, 0.00005, 0.0, 0.0),
		GRIPPER(0.5 , 0 , 0 , 0);			// FIXME - replace with real values
		
		private double p;
		private double i;
		private double d;
		private double f;
		
		private Pid(double p, double i, double d, double f) {
			this.p = p;
			this.i = i;
			this.d = d;
			this.f = f;
		}
		
		public double getP()
		{
			return p;
		}
		//
		public double getI()
		{
			return i;
		}
		//
		public double getD()
		{
			return d;
		}
		//
		public double getF()
		{
			return f;
		}
	}
	
//	public static final double MOTOR_P = 0.8;
//	public static final double MOTOR_I = 0.003;
//	public static final double MOTOR_D = 0.001;
//	public static final double MOTOR_F = 0.0;
//	
//	public static final double CONTROLLER_P = 9;
//	public static final double CONTROLLER_I = 0.008;
//	public static final double CONTROLLER_D = 0.005;
//	public static final double CONTROLLER_F = 0;
//	
//	public static final double CAMERA_P = 190;
//	public static final double CAMERA_I = 0.2;
//	public static final double CAMERA_D = 0.0;
//	public static final double CAMERA_F = 0;
	public static final double LIFT_ZERO_POSITION = -100;
	public static final double LIFT_SWITCH_POSITION = -15000;
	public static final double LIFT_SCALE_POSITION = -20000;
	public static final double LIFT_CLIMB_POSITION = -13000;
	public static final double LIFT_BAR_POSITION = -8000;
	public static final double LIFT_BOTTOM_POSITION = 0.0;
	public static final double LIFT_TOP_POSITION = -20500.0;
//	public static final double LIFT_RANGE = |SCALE_POSITION -
	public static final double LIFT_POSITION_THRESHOLD = 0.02;
//	public static final double GRIPPER_TILT_HORIZONTAL_POSITION =
	public static final double LIFT_secondsFromNeutralToFull = 0.5; 
	
	public static final double RAMP_LIFT_TIME = 5;
	public static final double RAMP_LIFT_SPEED = -400;
	public static final double RAMP_LIFT_MAX_CURRENT = 0.75;
	
	public static final double AUTONOMOUS_GYRO_TOLERANCE = 0.6;
	public static final double AUTONOMOUS_GYRO_RATE_TOLERANCE = 0.09;
	public static final double AUTONOMOUS_TURN_ANGLE = 90;
	//These Field constants are the distance from the side of the field
	// A -> W  are length stations
	// 1 -> 11 are width stations
	public static final int LIFT_ADJUST_POT = 3;
	public static final int GRIPPER_TILT_POT = 4;
	public static final int POSITION_KNOB = 0;
	public static final int OBJECTIVE_KNOB = 1;
	public static final int DELAY_KNOB = 2;
	public static final int GREEN_SWITCH = 1;
	public static final int BLUE_SWITCH = 2;
	public static final int RED_SWITCH = 3;
	
	public static final double FIELD_P11 = 306; //inches
	public static final double FIELD_P1 = 30;   // station P is the auto line
	public static final double FIELD_WIDTH = 324; // 27 feet
	public static final double FIELD_MIDLINE = 168;
	public static final double FIELD_SCALE_LENGTH = 180;
	public static final double FIELD_SCALE_WIDTH = 48;
	public static final double FIELD_NULL_WIDTH = 72;
	public static final double DISTANCE_SCALE_TO_FENCE = 72; // (fieldwidth - scalelength)/2
	public static final double CUBE_WIDTH = 13;
	public static final double FIELD_LEFT_START_POSITION = 30;
	public static final double FIELD_RIGHT_START_POSITION = 162;
	public static final double GRIPPER_LOCATION = 20;//Location of the gripper on the robot  //FIXME  
	// distance to the fence ultra sonic will read to match cube destination

	
	// ultrasonic characteristics
	public static final double ULTRASONICREAR_LOCATION = 7;
	public static final double ULTRASONIC_FRONT_LOCATION = 7;
	
	
	public static final double GRIPPER_TILT_SPEED = 0.3;
	public static final double GRIPPER_INFEED_SPEED = 0.4;
	public static final double GRIPPER_LAUNCH_SPEED = -0.5;
	public static final double GRIPPER_EJECT_SPEED = -0.3;
	public static final double GRIPPER_DRIBBLE_SPEED = -0.1;
	public static final double GRIPPER_TILT_VERTICAL_POSITION = 10;
	public static final double GRIPPER_TILT_DOWN_POSITION = 0;
	// Pixy camera characteristics
	public static final double LINE_PIXY_LOCATION= 15.6;
	
	// Gripper characteristics
	public static final double GRIPPER_SPITTIME = 1;//time cube needs to leave the gripper
	public static final double DISTANCE_TO_SPIT = 4; // how far cube goes from gripper when ejected
	public static final double LIFT_FLOOR_POSITION = 0;
	public static final double CUBEHANDLER_LENGTH = 12;

	public static final int LINE_COLOR_BLACK = 0;
	public static final int LINE_COLOR_RED = 1;
	public static final int LINE_COLOR_BLUE = 2;
	public static final int LINE_COLOR_WHITE = 3;
	public static final double LINE_SEARCH_SPEED = 300;
	
	public static final double AUTO_RIGHT_DRIVE_FORWARD_SPEED = 300;
	public static final double AUTO_LEFT_DRIVE_FORWARD_SPEED = 300;
	public static final double AUTO_RIGHT_DRIVE_REVERSE_SPEED = -300;
	public static final double AUTO_LEFT_DRIVE_REVERSE_SPEED = -300;
	
	
	public enum TapeColor 
	{
		NONE(-1),
		BLACK(9),
		WHITE(8),
		BLUE(7),
		RED(6);
		
		private int channel;
		
		private TapeColor(int digitalInputChannel)
		{
			channel = digitalInputChannel;
		}
		
		public int getDigitalInputChannel() 
		{
			return channel;
		}
	}
	
	public enum PNEUMATIC_CHANNEL
	{
		LOW_GEAR(0),
		HIGH_GEAR(1),
		PTO_ENGAGE(2),
		PTO_DISENGAGE(3);
		
		
		private int channel;
		private PNEUMATIC_CHANNEL(int channel_)
		{
			channel = channel_;
		}
		public int getChannel()
		{
			return channel;
		}
	}
	public enum CanId {

		LEFT_MASTER(10, true),
		LEFT_FOLLOWER(11, true),
		RIGHT_MASTER(20, false),
		RIGHT_FOLLOWER(21, false),
		LIFTER_MASTER(40, true),
		LEFT_GRIPPER(30, true),
		RIGHT_GRIPPER(31, false),
		LIFT_TILT(50,false);
		
		private int canId;
		
		private boolean isInverted;
		
		CanId(int id, boolean inverted) {
			canId = id;
			isInverted = inverted;
		}

		public int getCanId() {
			return canId;
		}
		
		public boolean isInverted()
		{
			return isInverted;
		}
	}


}
