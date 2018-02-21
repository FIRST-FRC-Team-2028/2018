package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutolineToScale extends CommandGroup {

	
	/**
	 * Drives from auto line to scale 
	 * drives position to put cube
	 * puts cube on scale
	 * 
	 * Param @ pidcontroller, drive, knobposition, left, gamedata
	 * sensors: ultrasonic rear, downpixy, gyro pidcontroller
	 */
	
	public AutolineToScale(PIDController pidcontroller, Drive drive,  Lift lift, Gripper gripper,
			Ultrasonic rearultrasonic, LineCamera linecam_,
			int knobposition, boolean left, String gamedata){
			boolean leftdriverswitch;
		LineCamera linecam = linecam_;
		leftdriverswitch = (knobposition > 7)? true: false;
//    	Drive forward to left or right of scale
//      Use Null line (white) as gauge
//		Dn = distance to null line = 132 inches
//    	D1 = distance from Auto line to mid line = 168 inches
//    	D2 = distance from Middle line to the end of null territory = 36 inches
//		Allowance for inaccuracy to avoid crossing the middle line is 6 inches.
//    	three steps: 
//		1) drive close to null line by distance (quickly)
//		      ~120 inches
//		2) drive across null line with sensor (slowly)  (point c)
//		3) drive past null line until edge of cube is 6 in away from mid line starting (point d)
//		dy = midline -6 - distance from front of cube to center of robot

//		dy = location of robot such that the the cube is 6 inches from the midline (y)
		double dy = Parameters.FIELD_MIDLINE -6 -Parameters.CUBEHANDLER_LENGTH;
		
//		by = the goal location of the cube on the scale (y)
		double by = Parameters.FIELD_MIDLINE - Parameters.FIELD_SCALE_WIDTH/2 +Parameters.CUBE_WIDTH/2 +3;
		
//		cy = location of the robot on the null line (y)
		double cy = Parameters.FIELD_MIDLINE -Parameters.FIELD_NULL_WIDTH/2 + Parameters.LINE_PIXY_LOCATION;
		
		
		addSequential(new DriveCommand(drive, 120));
		addSequential(new DriveAcrossLineCommand(drive, Parameters.TapeColor.WHITE, linecam )); 
    	addSequential(new DriveCommand(drive, dy-cy));

// 		Lift cube to scale ( while driving)
    	addParallel(new LiftCommand(lift, Parameters.LIFT_SCALE_POSITION));  //TODO Raise lift command
		
//    	Turn to face the scale
//      B is point to drop cube
//    	Bx= Field center +- (scalelength/2 + cubewidth/2 + safetymargin

//    	bx = target location of the cube on the scale
    	double dd = Parameters.FIELD_SCALE_LENGTH/2 + Parameters.CUBE_WIDTH + 3;
    	double bx = Parameters.FIELD_WIDTH/2;
    	bx+= (leftdriverswitch)?dd:-dd;

//    	dx = location of robot such that the cube is 6 inches away from the midline (x)
    	double dx = (leftdriverswitch)? Parameters.FIELD_LEFT_START_POSITION:Parameters.FIELD_RIGHT_START_POSITION;
    	
//      angle = atan( (bx-dx)/(by-dy))
    	double turnangle = Math.atan2((bx-dx),(by-dy));
    	addSequential(new RotateCommand(drive, pidcontroller, turnangle)); 
    	
    	
//    	drive to scale 
//    	uses rear facing ultrasonic to determine position C from field fence at the current orientation
//    	Cx = fieldwidth/2 - scalelength/2 + CubeLength/2 - gripperLocation - rearUltrasonic location
//    	Distance to wall is Cx/sin(angle)
    	
//    	cx the location the robot is to drive to so as to put the cube on the scale
    	double cx = Parameters.FIELD_MIDLINE - Parameters.FIELD_SCALE_LENGTH/2 - (Parameters.DISTANCE_TO_SPIT +  
				 Parameters.ULTRASONICREAR_LOCATION);
    	dd = cx/Math.sin(turnangle);		
    	addSequential( new DriveUltrasonicCommand(drive, rearultrasonic, dd ));
    	

//    	Deposit cube on scale
    	addSequential(new PlaceCubeCommand(gripper, Parameters.GRIPPER_EJECT_SPEED));
    	
	}
}
