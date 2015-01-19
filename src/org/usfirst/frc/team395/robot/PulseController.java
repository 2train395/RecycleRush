package org.usfirst.frc.team395.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;


public class PulseController {

	// Instance Variables
	private Talon motor;		// Motor controller used for pulsing
	private Timer timer;		// Timer used to control pulse time
	private double pulseTime;	// Amount of time per pulse
	private boolean pulsing;	// Tracks whether or not pulsing is happening
	private double speed;		// Sets direction & magnitude of pulse speed
	
	/**
	 * Constructor for new Pulse Controllers -- note the pulse time can be
	 * defined after initial construction using setPulsetime(double) for
	 * pulse times up to a certain upper limit. Initially PulseControllers
	 * are set to be off.
	 * @param m
	 */
	public PulseController(Talon m){
		
		motor = m;
		timer = new Timer();
		this.setPulseTime(0.05);
		pulsing = false;
		speed = 0.0;
		
	}
	
	/**
	 * This method sets the time for the pulse -- has a default value set in 
	 * the constructor.
	 * @param t pulse time in milliseconds
	 */
	public void setPulseTime(double t){
		if (t <= 0.1){
			pulseTime = t;
		}
		else {
			pulseTime = 0.05;
		}
	}
	
	/**
	 * This method sets the motor speed (+/- 1.0) if the user has pressed the 
	 * 'up' or 'down' buttons controlling the motor. Once the motor starts it
	 * runs for the duration of 'pulseTime' before shutting off. If the user
	 * is still holding down the button, the motor will not turn on again until
	 * the user lets go and then presses the button again.
	 * @param upButtonVal boolean taken from the rawButton value on the
	 * joystick of choice representing 'up' (or 'out', etc)
	 * @param downButtonVal boolean taken from the rawButton value on the
	 * joystick of choice representing 'down' (or 'in', etc)
	 */
	public void pulseControl(boolean upButtonVal, boolean downButtonVal){
		
		if (upButtonVal	&& !downButtonVal && !pulsing) {
			pulsing = true;
			speed = 1.0;
			timer.start();
		}
		if (!upButtonVal && downButtonVal && !pulsing) {
			pulsing = true;
			speed = -1.0;
			timer.start();
		}
		
		if (timer.get() > pulseTime){
			
			speed = 0.0;
			
		if(!upButtonVal && !downButtonVal){
				timer.stop();
				timer.reset();
				pulsing = false;
			}
		}
		
		if (pulsing) { 
			motor.set(speed);
		}
		else {
			motor.set(0.0);
			timer.stop();
		}
	}
	
	/**
	 * This [overloaded] method sets the motor speed (+/- 1.0) if the user
	 * has pressed the 'up' or 'down' triggers (read as axis) controlling the
	 * motor. Once the motor starts it runs for the duration of 'pulseTime' 
	 * before shutting off. If the user is still holding down the button, the
	 * motor will not turn on again until the user lets go and then presses
	 * the button again.
	 * @param upAxis double taken from the rawAxis value on the
	 * joystick of choice representing 'up' (or 'out', etc)
	 * @param downAxis double taken from the rawAxis value on the
	 * joystick of choice representing 'down' (or 'in', etc)
	 */
	public void pulseControl(double upAxis, double downAxis){
		
		if ((upAxis > 0.5) && (downAxis < 0.5) && !pulsing) {
			pulsing = true;
			speed = 1.0;
			timer.start();
		}
		if ((upAxis < 0.5) && (downAxis > 0.5) && !pulsing) {
			pulsing = true;
			speed = -1.0;
			timer.start();
		}
		
			
		if (timer.get() > pulseTime){
			
			speed = 0.0;
			
			if((upAxis < 0.5) && (downAxis < 0.5)){
				timer.stop();
				timer.reset();
				pulsing = false;
			}
		}
		
		if (pulsing) { 
			motor.set(speed);
		}
		else {
			motor.set(0.0);
			timer.stop();
		}
	}
	
}
