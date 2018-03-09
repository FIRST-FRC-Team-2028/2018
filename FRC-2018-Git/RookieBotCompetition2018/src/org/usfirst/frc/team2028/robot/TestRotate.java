package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestRotate extends TimedRotateCommand{

	static double[] speeds = {50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550};
	static double[] angles= {15, 30, 45, 70, 90, 120};
	Command rotate;
	
    public TestRotate(double speed, double angle, Drive drive, int speedknob, int angleknob, Gyro gyro) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
//    	super(speed, angle, drive, pidcontroller);
    	super(speeds[speedknob-1], angles[angleknob-1], drive);
    }
}