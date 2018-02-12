package org.usfirst.frc.team2028.robot;

public class Parameters {

	/**
	 * Flags to control condition hardware availability between compeition and rookie/practice bot
	 */
	public static final boolean GYRO_AVAILABLE = false;
	
	
	public static final boolean RIGHT_PHASE = false;
	public static final boolean LEFT_PHASE = true;
	public static final boolean LEFT_DRIVE_INVERTED = false;
	public static final boolean LEFT_GRIPPER_INVERTED = true;
	
	public static final double MOTOR_P = 0.8;
	public static final double MOTOR_I = 0.003;
	public static final double MOTOR_D = 0.001;
	public static final double MOTOR_F = 0.0;
	
	public static final double CONTROLLER_P = 9;
	public static final double CONTROLLER_I = 0.008;
	public static final double CONTROLLER_D = 0.005;
	public static final double CONTROLLER_F = 0;
	
	public static final double CAMERA_P = 190;
	public static final double CAMERA_I = 0.2;
	public static final double CAMERA_D = 0.0;
	public static final double CAMERA_F = 0;
	
	public static final int LOW_GEAR = 0;
	public static final int HIGH_GEAR = 1;
	public static final double ZERO_POSITION = 0;
	public static final double SWITCH_POSITION = 6500;
	public static final double SCALE_POSITION = 14500;
//	public static final double LIFT_RANGE = |SCALE_POSITION -
	public static final double LIFT_POSITION_THRESHOLD = 0.02;
	
	public static final double AUTONOMOUS_GYRO_TOLERANCE = 0.6;
	public static final double AUTONOMOUS_GYRO_RATE_TOLERANCE = 0.09;
	public static final double AUTONOMOUS_TURN_ANGLE = 90;
	//These Field constants are the distance from the side of the field
	// A -> W  are length stations
	// 1 -> 11 are width stations
	public static final double FIELD_P11 = 306; //inches
	public static final double FIELD_P1 = 30;   //P is the auto line
	
	public static final double CUBEHANDLER_LENGTH = 12;
	public static final double DISTANCE_SCALE_TO_FENCE = 72; // (fieldwidth - scalelength)/2
	// distance from the base of the outrigger ultra sonic will read to match cube destination
	public static final double DISTANCE_TO_SWITCH_FENCE = 2.25;
	// distance to the fence ultra sonic will read to match cube destination
	public static final double ROBOT_LENGTH = 32.75;

	public static final double ULTRASONICREAR_LOCATION = 7;
	
	public static final double GRIPPER_SPITTIME = 1;//time cube needs to leave the gripper
	public static final double DISTANCE_TO_SPIT = 18; // distance from cube destination

	public enum PNEUMATIC_CHANNEL
	{
		OUTRIGGER_DEPLOY(2),
		OUTRIGGER_OFF(3);
		
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
		LIFTER_MASTER(14, false),
		LIFTER_FOLLOWER(15, false),
		LEFT_LIFT(97,false),
		RIGHT_LIFT(96, false),
		LEFT_GRIPPER(16, true),
		RIGHT_GRIPPER(17, false);

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
