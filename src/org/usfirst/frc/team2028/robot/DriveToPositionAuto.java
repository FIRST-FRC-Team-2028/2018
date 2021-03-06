package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveToPositionAuto extends CommandGroup {

    public DriveToPositionAuto(PIDController pidcontroller, Drive drive, double pos, double distance, Ultrasonic ultrasonic) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.
    	SmartDashboard.putBoolean("DriveCommandGROUP", true);
    	requires(drive);
//    	addSequential(new DriveCommand(drive, pos));
//    	addSequential(new DriveCommand(drive, pos));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new RotateCommand(drive, pidcontroller, 90));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new DriveCommand(drive, pos));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new RotateCommand(drive, pidcontroller, 180));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new DriveCommand(drive, pos));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new RotateCommand(drive, pidcontroller, 270));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new DriveCommand(drive, pos));
//    	addSequential(new WaitCommand(5));
//    	addSequential(new RotateCommand(drive, pidcontroller, 0));
    	addSequential(new DriveUltrasonicCommand(drive, ultrasonic, distance));
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
