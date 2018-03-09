package org.usfirst.frc.team2028.robot;

import org.usfirst.frc.team2028.robot.Parameters.TapeColor;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive straight at a desired speed until line camera detects the desired line
 * Similar to DriveAcrossLineCommand
 */
public class LineTest extends DriveAcrossLineCommand {
    private final int speed;
    
    public LineTest(Drive drive, LineCamera camera, TapeColor line_type, int speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super(drive, line_type, camera);
    	
    	switch(speed)
    	{
    	case 1: 
    		this.speed = 100;
    		break;
    	case 2:
    		this.speed = 300;
    		break;
    	case 3:
    		this.speed = 500;
    		break;
    	case 4:
    		this.speed = 700;
    		break;
    	default:
    		this.speed = 0;
    		break;
    	}
    	requires(drive);
    }

    // Called just before this Command runs the first time
    // used super's version of protected void initialize() {
    //}

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		drive.go();
		drive.set(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    // use super's isFinished method

    // Called once after isFinished returns true
    // use super's protected void end() {
    //}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    // use super's protected void interrupted() {
    //}
}
