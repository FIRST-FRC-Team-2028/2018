package org.usfirst.frc.team2028.robot;

import org.usfirst.frc.team2028.robot.Parameters.TapeColor;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TestGroup extends CommandGroup {

    public TestGroup(int objectiveknob, Drive drive, int positionknob, int delayknob,
    		TapeColor line_type, Lift lift, Gripper gripper, Gyro gyro, LineCamera camera,
    		double tiltpot) {
    	
    	switch(objectiveknob)
    	{
    	case 1:
    		addSequential(new TestRotate(0, 0, drive, positionknob, delayknob, gyro));
    		break;
    	case 2:
        	addSequential(new LineTest(drive, camera, line_type, positionknob));
        	break;
    	case 3:
    		addSequential(new TestAngleMotor(gripper, positionknob, delayknob, tiltpot));
    		break;
        default:
        	break;
        
    	}
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

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
