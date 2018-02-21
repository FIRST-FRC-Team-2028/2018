package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LiftRampCommand extends Command {
	
	Drive drive;
	
	private boolean started;
	
    public LiftRampCommand(Drive drive) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	requires(drive);
    	this.drive = drive;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	drive.setPTOLow();
    	setTimeout(Parameters.RAMP_LIFT_TIME);
    }
    
    

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	drive.set(Parameters.RAMP_LIFT_SPEED);
    	drive.go();
    	started = true;
    }

    public boolean isStarted()
    {
    	return started;
    }
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	drive.stop();
    	drive.go();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
