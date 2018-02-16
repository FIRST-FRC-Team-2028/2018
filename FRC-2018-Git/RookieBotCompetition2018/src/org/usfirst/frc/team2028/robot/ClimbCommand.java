package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimbCommand extends Command {
	Lift lift;
    public ClimbCommand(Lift lift) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(lift);
		this.lift = lift;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	lift.setPosition(Parameters.LIFT_CLIMB_POSITION);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(lift.getPosition() > Parameters.LIFT_CLIMB_POSITION-4 &&
    			lift.getPosition() < Parameters.LIFT_CLIMB_POSITION+10){
        return true;
    	}else
    	{
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
//    	lift.engageRatchet();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
}