package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoOption8 extends CommandGroup {
	

    public AutoOption8(PIDController cameracontroller, PIDController pidcontroller, Drive drive, Lift lift, Gripper gripper, Ultrasonic rearultrasonic,
    		LineCamera linecamera, double delay, int positionknob, boolean leftscale, String gamedata) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

    	//Drive and place a cube on scale
    	addSequential(new AutoOption4(pidcontroller, drive, lift, gripper, rearultrasonic, linecamera,
    			delay, positionknob, leftscale, gamedata));
    	
    	//back up and reset gripper
    	addSequential(new AfterDepositCommand(drive, lift));
    	
    	//turns toward the alliance wall
    	addSequential(new RotateCommand(drive, pidcontroller, 180));
    	
    	//drive out of null zone
    	addSequential(new DriveCommand(drive, 10));
    	
    	//turn towards the switch
    	addSequential(new RotateCommand(drive, pidcontroller, (leftscale)? 180-45: 180+45));
    	
    	//get the cube
    	addSequential(new DriveToCubeCommand(drive, rearultrasonic, cameracontroller, gripper));
    	
        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
