package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoTest4 extends CommandGroup {

	
	/**
	 * Test after having crossed auto line 
	 * drives position to put cube
	 * puts cube on scale
	 * 
	 * Param @ pidcontroller, drive, knobposition, left, gamedata
	 * sensors: ultrasonic rear, downpixy, gyro pidcontroller
	 */
	public AutoTest4(PIDController pidcontroller, Drive drive, Lift lift,  Ultrasonic rearultrasonic,
			int knobposition, boolean left, String gamedata){
			boolean leftdriverswitch;
		leftdriverswitch = (knobposition > 7)? true: false;
//    	Drive forward to left or right of scale
//      Use Null line (white) as gauge
//    	D1 = distance from Auto line to mid line = 168 inches
//    	D2 = distance from Middle line to the end of null territory = 36 inches
//		Allowance for inaccuracy to avoid crossing the middle line is 6 inches.
//    	two steps: 
//		2) drive across null line with sensor (slowly)
//		3) drive past null line by sufficient room to deposit cube
//		By = midline - scalewitch/2 + cubewidth/2 + gap
//		By = 168 - 48/2 + 13/2 + 3
		
		double DPastNullLine = 10; // once we determine where the downward 
//		pixy is to be the DPastNullLine = Dn/2 -(location of pixy + AllowanceForInaccuary)
		
		
//		addSequential(new DriveAcrossLineCommand(drive, )); //TODO DriveAcrossLineCommand
    	addSequential(new DriveCommand(drive, DPastNullLine));
		
//    	Turn to face the scale
    	double turnangle = (leftdriverswitch)? 90 : -90;
    	addSequential(new RotateCommand(drive, pidcontroller, turnangle));

// 		Lift cube to scale
//    	addSequential(new LiftCommand(lift, pos));  //TODO Raise lift command 
    	
    	
//    	drive to scale 
//    	uses rear facing ultrasonic to determine position from field fence
//    	Cx = fieldwidth/2 - scalelength/2 - standoffdistance
//    	location of ultrasonic sensor URX = Cx - location of sensor on robot
//  	Rear UltraSonic location/2
    	
    	double cx = 324/2 - 180/2 - (Parameters.DISTANCE_TO_SPIT +  
				 Parameters.ULTRASONICREAR_LOCATION);
    	addSequential( new DriveUltrasonicCommand(drive, rearultrasonic, cx ));
    	

//    	Deposit cube on scale
//    	addSequential(new PlaceCubeCommand())//TODO place cube command
    	
    	//reset Lift position
    	addSequential(new ResetGripperPosition(drive, lift));
    	
	}
}

