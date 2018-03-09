package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestAngleMotor extends Command {

	Gripper gripper;
	double Pvalue;
	double Ivalue;
	double angle;
	
	double[] P = {0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2};
	double[] I = {.0001, .0005, .001, .005, .006, .007, .008, .009, .01, .011};
	
    public TestAngleMotor(Gripper gripper, int Pknob, int Iknob, double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.gripper = gripper;
    	Pvalue = P[Pknob];
    	Ivalue = I[Iknob];
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		gripper.tiltTo(((angle+1))*(50));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
