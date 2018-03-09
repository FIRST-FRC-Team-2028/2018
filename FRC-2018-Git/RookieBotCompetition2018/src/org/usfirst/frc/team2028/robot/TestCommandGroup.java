package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class TestCommandGroup extends CommandGroup {
	
	Drive drive;
	
	PIDController pidcontroller;
	
    public TestCommandGroup(Drive drive, int objectiveknob, int delayknob, Gyro gyro) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.
    	addSequential(new TestRotate(0, 0, drive, objectiveknob, delayknob, gyro));
//    	addSequential(new RotateCommand(drive, pidcontroller, 90));
//    	addSequential(new LineTest(drive, null, null, m_currentCommandIndex));
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
