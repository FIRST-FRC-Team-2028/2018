package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2028.robot.Parameters.PNEUMATIC_CHANNEL;

/**
 * This is the subsystem constructor and generally necessary code for the ramp subsystem.
 */
public class Ramp extends Subsystem {

	private boolean state_of_outriggers = false;

	private DoubleSolenoid solenoid = null; 

	public Ramp()
	{
		if (Parameters.RAMP_AVAILABLE) {
			solenoid = new DoubleSolenoid(
					PNEUMATIC_CHANNEL.OUTRIGGER_DEPLOY.getChannel() ,
					PNEUMATIC_CHANNEL.OUTRIGGER_OFF.getChannel() ); 
		}
	}
	/**
	 * This method turns on the outriggers.
	 */

	public void deployOutriggers()
	{
		if(Parameters.RAMP_AVAILABLE){
			solenoid.set(DoubleSolenoid.Value.kForward);
			state_of_outriggers = true;
		}
	}

	/**
	 * This method turns off the outrigggers.
	 */
	public void turnOffOutrigger()
	{
		if(Parameters.RAMP_AVAILABLE){
			solenoid.set(DoubleSolenoid.Value.kOff);
		}
	}

	public void retractOutrigger()
	{
		if(Parameters.RAMP_AVAILABLE){
			solenoid.set(DoubleSolenoid.Value.kReverse);
		}
	}


	/**
	 * Gets whether or not the outriggers are employed.
	 * @return
	 */
	public boolean getStateOfOutriggers()
	{
		if(Parameters.RAMP_AVAILABLE){
			return state_of_outriggers;
		}
		return true;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}


}

