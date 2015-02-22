////  ___ _______        _       _____   ____  ____   ____ _______ _____ _____  _____ 
//// |__ \__   __|      (_)     |  __ \ / __ \|  _ \ / __ \__   __|_   _/ ____|/ ____|
////    ) | | |_ __ __ _ _ _ __ | |__) | |  | | |_) | |  | | | |    | || |    | (___  
////   / /  | | '__/ _` | | '_ \|  _  /| |  | |  _ <| |  | | | |    | || |     \___ \ 
////  / /_  | | | | (_| | | | | | | \ \| |__| | |_) | |__| | | |   _| || |____ ____) |
//// |____| |_|_|  \__,_|_|_| |_|_|  \_\\____/|____/ \____/  |_|  |_____\_____|_____/ 

package org.usfirst.frc.team395.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Gyro;

public class Robot extends IterativeRobot {

	// DRIVE
	RobotDrive robotDrive;
	final int frontLeftChannel	= 1;
	final int rearLeftChannel	= 2;
	final int frontRightChannel	= 3;
	final int rearRightChannel	= 4;
	
	// LIFT
	Talon lift;
	PulseController liftPulser;
	Encoder liftEncoder;
	DigitalInput liftTopLimitSwitch;
	DigitalInput liftBottomLimitSwitch;
	SafePIDOutput pidOutput;
	PIDController liftPID;
	double liftTargetPosition;
	final int LIFT_ENCODER_DIO_CHANNEL_A = 0;
	final int LIFT_ENCODER_DIO_CHANNEL_B = 1;
	final int LIFT_TOP_LIMIT_DIO_CHANNEL = 2;
	final int LIFT_BOTTOM_LIMIT_DIO_CHANNEL = 3;
	final int LIFT_MOTOR_CHANNEL = 5;
	final double LIFT_PID_GAIN_P = 0.001; 
	final double LIFT_PID_GAIN_I = 0.0000;   // probably don't need I for positional
	final double LIFT_PID_GAIN_D = 0.0002;
	final double LIFT_SPEED = 0.65;
    
	// GRIPPER 
	Talon gripper;
	PulseController gripPulser;
	final int GRIPPER_MOTOR_CHANNEL = 6;
	final double GRIPPER_PULSE_TIME = 0.05;
	final double GRIPPER_SPEED = 0.80;
	
	// ROLLERS
	Relay rightRoller;
	Relay leftRoller;
	Talon rollerArm;
	final int RIGHT_ROLLER_RELAY_CHANNEL = 0;
	final int LEFT_ROLLER_RELAY_CHANNEL = 1;
	final int ROLLER_ARM_MOTOR_CHANNEL = 9;
	  
	
	// JOYSTICKS
	Joystick driveStick;
	Joystick xboxController;
	final int driveStickChannel = 1;
	final int XBOX_CONTROLLER_CHANNEL = 2;
	final int LIFT_DOWN_BUTTON = 1;
	final int LIFT_UP_BUTTON = 4;
	final int GRIPPER_IN_BUTTON = 3;
	final int GRIPPER_OUT_BUTTON = 2;
	final int LEFT_OUT_BUTTON = 5;
	final int RIGHT_OUT_BUTTON = 6;
	final int LEFT_IN_AXIS = 2;
	final int RIGHT_IN_AXIS = 3;
	final int ROLLER_ARM_OPEN = 9;
	final int ROLLER_ARM_CLOSE = 10;
	
	// ANALOG
	Gyro gyro;
	final int GYRO_CHANNEL =  0;
	final double GYRO_SENSITIVITY = 0.007;
	
	public void robotInit() {
		
		// DRIVE
		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
		robotDrive.setExpiration(0.1); 
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
		robotDrive.setSafetyEnabled(true); 
		
		// lift
		liftTopLimitSwitch = new DigitalInput(LIFT_TOP_LIMIT_DIO_CHANNEL);
		liftBottomLimitSwitch = new DigitalInput(LIFT_BOTTOM_LIMIT_DIO_CHANNEL); 
		lift = new Talon(LIFT_MOTOR_CHANNEL);
		liftEncoder = new Encoder(LIFT_ENCODER_DIO_CHANNEL_A, LIFT_ENCODER_DIO_CHANNEL_B);
		liftEncoder.setReverseDirection(true);
		liftEncoder.setIndexSource(liftBottomLimitSwitch);
		liftEncoder.reset();       
        pidOutput = new SafePIDOutput(lift, liftTopLimitSwitch, liftBottomLimitSwitch);
        liftPID = new PIDController(LIFT_PID_GAIN_P, LIFT_PID_GAIN_I, LIFT_PID_GAIN_D, liftEncoder, pidOutput);
        liftTargetPosition = 0.0;
	   
		//Gripper
		gripper = new Talon(GRIPPER_MOTOR_CHANNEL);
		gripPulser = new PulseController(gripper);
	    gripPulser.setPulseTime(GRIPPER_PULSE_TIME);
	    
	    //rollers
	    leftRoller = new Relay(LEFT_ROLLER_RELAY_CHANNEL);
		rightRoller = new Relay(RIGHT_ROLLER_RELAY_CHANNEL);
		rollerArm = new Talon(ROLLER_ARM_MOTOR_CHANNEL);
		
		// JOYSTICK
		driveStick = new Joystick(driveStickChannel);
		xboxController = new Joystick(XBOX_CONTROLLER_CHANNEL);
		
		//ANALOG
		gyro = new Gyro(GYRO_CHANNEL);
		gyro.setSensitivity(GYRO_SENSITIVITY);
	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {

		manualDrive();
		
		if (xboxController.getRawButton(LIFT_UP_BUTTON) || xboxController.getRawButton(LIFT_DOWN_BUTTON)) {
			// disable the pid
			liftPID.disable();
			if (xboxController.getRawButton(LIFT_UP_BUTTON)) {
				
				pidOutput.setOutput(LIFT_SPEED);
				
			} 
			
			else {
				pidOutput.setOutput(-0.5*LIFT_SPEED);
			}
			
			liftTargetPosition = (double)liftEncoder.get();
		} 
		
		else {
			//pidOutput.setOutput(0);
			pidOutput.hold();
			liftPID.enable();
			liftPID.setSetpoint(liftTargetPosition);
		}
	
		//gripPulser.pulseControl(xboxController.getRawButton(GRIPPER_OUT_BUTTON), xboxController.getRawButton(GRIPPER_OUT_BUTTON));
		
		if(xboxController.getRawButton(GRIPPER_OUT_BUTTON)){
			gripper.set(GRIPPER_SPEED);
		}
		else if(xboxController.getRawButton(GRIPPER_IN_BUTTON)){
			gripper.set(-GRIPPER_SPEED);			
		}
		else{
			gripper.set(0.0);
		}
		
		SmartDashboard.putBoolean("Gripper Out Button", xboxController.getRawButton(GRIPPER_OUT_BUTTON));
		SmartDashboard.putBoolean("Gripper In Button", xboxController.getRawButton(GRIPPER_IN_BUTTON));
		SmartDashboard.putBoolean("Limit Switch Upper", liftTopLimitSwitch.get());
		SmartDashboard.putBoolean("Limit Switch Lower", liftBottomLimitSwitch.get());
		SmartDashboard.putNumber("Lift Encoder Value", liftEncoder.get());
		SmartDashboard.putData("PID Controller", liftPID);
		SmartDashboard.putNumber("PID Controller Value", liftPID.get());
		
		Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles?
        
	/*
	//SAME DIRECTION ROLLING
		if(xboxController.getRawButton(LEFT_OUT_BUTTON)){
			leftRoller.set(Relay.Value.kReverse);//INVERTED MOTOR
	 	}
		else if(xboxController.getRawAxis(LEFT_IN_AXIS) > 0.5){
			leftRoller.set(Relay.Value.kForward); //INVERTED MOTOR
		}
		else{
			leftRoller.set(Relay.Value.kOff);
		}
		
		if(xboxController.getRawButton(RIGHT_OUT_BUTTON)){
			rightRoller.set(Relay.Value.kReverse);//INVERTED MOTOR
		}
		else if(xboxController.getRawAxis(RIGHT_IN_AXIS) > 0.5){
			rightRoller.set(Relay.Value.kForward);
		}
		else{
			rightRoller.set(Relay.Value.kOff);
		}
	//ROLLER ARM 
		if(xboxController.getRawButton(ROLLER_ARM_OPEN)){
			rollerArm.set(1.0);
		}
		else if(xboxController.getRawButton(ROLLER_ARM_CLOSE)){
			rollerArm.set(-1.0);
		}
		else{
			rollerArm.set(0.0);
		}*/
	}
	
	public void testPeriodic() {
		
	}
	
	public void manualDrive() {
				
		robotDrive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), driveStick.getZ(), 0);
	}
		
}