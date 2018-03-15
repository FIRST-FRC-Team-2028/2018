package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DeployRampCommand extends Command {

	private Drive drive;

	private boolean finished = false;
	
	public DeployRampCommand(Drive drive) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(drive);
		this.drive = drive;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		drive.setLowGear();
		if(Parameters.RAMP_AVAILABLE){
			drive.setPTOLow();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if(Parameters.RAMP_AVAILABLE){
			SmartDashboard.putNumber("PTO_current", drive.getMaximumMotorCurrent());
			drive.set(Parameters.RAMP_LIFT_SPEED);
			drive.go();
			if(drive.getMaximumMotorCurrent() > Parameters.RAMP_LIFT_MAX_CURRENT)
			{
				finished = true;
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return finished;
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