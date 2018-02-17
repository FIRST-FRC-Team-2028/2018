package org.usfirst.frc.team2028.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2028.robot.Parameters.PNEUMATIC_CHANNEL;

/**
 * This is the subsystem constructor and generally necessary code for the ramp subsystem.
 */
public class Ramp extends Subsystem {

	private boolean state_of_outriggers = false;
	
	private DoubleSolenoid solenoid = new DoubleSolenoid(
			PNEUMATIC_CHANNEL.OUTRIGGER_DEPLOY.getChannel() ,
			PNEUMATIC_CHANNEL.OUTRIGGER_OFF.getChannel() ); 
	
	/**
	 * This method turns on the outriggers.
	 */
	
    public void deployOutriggers()
    {
    	solenoid.set(DoubleSolenoid.Value.kForward);
    	state_of_outriggers = true;
    }
    
    /**
     * This method turns off the outrigggers.
     */
    
    public void turnOffOutrigger()
    {
    	solenoid.set(DoubleSolenoid.Value.kOff);
    }
    
    /**
     * Gets whether or not the outriggers are employed.
     * @return
     */
    public boolean getStateOfOutriggers()
    {
    	return state_of_outriggers;
    }

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
    
    
}

