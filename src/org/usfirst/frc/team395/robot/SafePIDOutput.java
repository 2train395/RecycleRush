package org.usfirst.frc.team395.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;

public class SafePIDOutput implements PIDOutput {
	
	private DigitalInput m_bottomLimitSwitch;
	private DigitalInput m_topLimitSwitch;
	private Talon m_output;
	private boolean m_pidEnabled = false;
	/**
	 * The current output of this PIDOutput
	 */
	private double m_currentOutput = 0;
	
	public SafePIDOutput(Talon controller, DigitalInput upper, DigitalInput lower){
		m_bottomLimitSwitch = lower;
		m_topLimitSwitch = upper;
		m_output = controller;
	}
	
	/**
	 * Writes a motor value to the Talon if PID is enabled.
	 */
	public void pidWrite(double output) {
		if (!m_pidEnabled) {
			return;
		}
		
		setSpeed(-1*output);
	}
	
	/**
	 * Get the current output of this PIDOutput
	 * @return {Double} The current output value
	 */
	public double getCurrentOutput() {
		return m_currentOutput;
	}
	
	public void setOutput(double output) {
		m_pidEnabled = false;
		setSpeed(output);
	}
		
	public void hold() {
		m_pidEnabled = true;
	}
	
	private void setSpeed(double output) {
		double actualSpeed;
        
        
        if (m_bottomLimitSwitch.get()) {
            // if we are too low only let us go up
            if (output < 0) {
                actualSpeed = 0.0;
            } 
            else {
                actualSpeed = output;
            }
        }
        else if (!m_topLimitSwitch.get()) {
            // if we are too high only let us go down
            if (output > 0) {
                actualSpeed = 0.0;
            } 
            else {
                actualSpeed = output;
            }
        }
        else {
            // We're okay to go!
            actualSpeed = output;
        }
        
        m_output.set(actualSpeed);
        m_currentOutput = actualSpeed;
	}

}
