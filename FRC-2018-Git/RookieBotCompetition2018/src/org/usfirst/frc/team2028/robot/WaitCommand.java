package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WaitCommand extends Command {
	double initialtime;
	double settime;
	Timer timer;
    public WaitCommand(double time_) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	settime = time_;
    	timer = new Timer();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	initialtime = timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(timer.getFPGATimestamp()-initialtime > settime)
        {
        	return true;
        }
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
