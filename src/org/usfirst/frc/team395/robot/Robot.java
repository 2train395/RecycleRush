package org.usfirst.frc.team395.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

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
	final int liftEncoderDIO_ChannelA = 0;
	final int liftEncoderDIO_ChannelB = 1;
	final int liftMotorChannel = 5;
	final double liftPulseTime = 0.05;
	
	// GRIPPER 
	Talon gripper;
	PulseController gripPulser;
	final int gripperMotorChannel = 6;
	final double gripperPulseTime = 0.05;
	
	// JOYSTICKS
	Joystick driveStick;
	Joystick xboxController;
	
	final int driveStickChannel = 1;
	final int xBoxControllerChannel = 2;
	
	final int liftDownButton = 1;
	final int liftUpButton = 4;
	final int gripperOutButton = 3;
	final int gripperInButton = 2;
	
	public void robotInit() {
		

		// DRIVE
		robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
		robotDrive.setExpiration(0.1); // Not sure what this does
		robotDrive.setInvertedMotor(MotorType.kFrontRight, true);
		robotDrive.setInvertedMotor(MotorType.kRearRight, true);
		robotDrive.setSafetyEnabled(true); // not sure what this does
		
		// lift
		lift = new Talon(liftMotorChannel);
		liftEncoder = new Encoder(liftEncoderDIO_ChannelA, liftEncoderDIO_ChannelB);
		liftEncoder.reset();
		liftPulser = new PulseController(lift);
		liftPulser.setPulseTime(liftPulseTime);
		
		//Gripper
		gripper = new Talon(gripperMotorChannel);
		gripPulser = new PulseController(gripper);
	    gripPulser.setPulseTime(gripperPulseTime);
		
		// JOYSTICK
		driveStick = new Joystick(driveStickChannel);
		xboxController = new Joystick(xBoxControllerChannel);
		

	}

	public void autonomousPeriodic() {

	}

	public void teleopPeriodic() {

		manualDrive();
		liftPulser.pulseControl(xboxController.getRawButton(liftUpButton),
								  xboxController.getRawButton(liftDownButton));
		gripPulser.pulseControl(xboxController.getRawButton(gripperOutButton),
								  xboxController.getRawButton(gripperInButton));
		Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles?
		
		SmartDashboard.putNumber("Lift Encoder Value", liftEncoder.get());
	}

	public void testPeriodic() {
		
	}
	
	public void manualDrive() {
				
		robotDrive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), driveStick.getZ(), 0);
	}
		
}
