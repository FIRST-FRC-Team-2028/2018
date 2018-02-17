package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.XboxController;

/**
 *
 */
public class JoystickDrive extends Command {

	private XboxController controller;
	
    public JoystickDrive(XboxController con) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	controller = con;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished()
    {
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
