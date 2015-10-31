package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousOld implements Runnable {
	Timer autoTimer;
	DigitalInput autoEye;
	Robot theBot;
	
	int step;
	int autoMode;
	
	public AutonomousOld(Robot robot){
		theBot = robot;
		autoEye = new DigitalInput(12); //not necessarily 12
		autoTimer = new Timer();
		step = 0;
		theBot.BackL.setGearPID(false);
		theBot.FrontL.setGearPID(false);
		theBot.BackR.setGearPID(false);
		theBot.FrontR.setGearPID(false);
		autoMode = 0;
	}
	
	public void resetDriveValues(){
		theBot.robotDrive.mecanumDrive_Cartesian(0,0,0,0);
		}
	
	public void nextAutoStep(){
		autoTimer.reset();
		++step;
		}
	
	public void driveTime(long time, double x, double y, double rotate){
		theBot.robotDrive.mecanumDrive_Cartesian(x, y, rotate, 0);
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
		}
	}
	
	public void driveDistance(long count, double x, double y, double rotate){
		while (theBot.FrontL.get() < Math.abs(count) || theBot.FrontR.get() < Math.abs(count)){
			theBot.robotDrive.mecanumDrive_Cartesian(x,y,rotate, 0);
		}
	}
	
	public void rotate(float degrees, double rotate){
		if (theBot.navx.getYaw() > degrees){
			while (theBot.navx.getYaw() > degrees){
				theBot.robotDrive.mecanumDrive_Cartesian(0,0, -rotate, 0);
			}
		} else {
			while (theBot.navx.getYaw() < degrees){
				theBot.robotDrive.mecanumDrive_Cartesian(0,0, rotate, 0);
			}
		}
	}
	 

	public void moveToZoneAuto(){
		
		driveTime(10, 0, .25, 0);
		resetDriveValues();
		}
	

		public void moveOneToteAuto(){
			switch(step){
			
			case 1:
			rotate(90, .25);
			resetDriveValues();
			nextAutoStep();
			break;
			
			case 2: 
			driveTime(1000, 0, .25, 0);
			resetDriveValues();
			nextAutoStep();
			break;
			}
		}
				
		public void changeAuto(){
			autoMode += 1;
			if (autoMode > 2){
				autoMode = 0;
			}
			SmartDashboard.putNumber("Auto Mode", autoMode);
		}
		
		public void setMode(int mode){
			switch(mode){
			
			case 0:
				resetDriveValues();
				break;
			
			case 1:
				moveToZoneAuto();
				autoMode = 0;
				break;
				
			case 2:
				moveOneToteAuto();
				break;
			
			}
		}
		
		

		@Override
		public void run() {
			while (true){
				setMode(autoMode);
			}
			
			
		}
		

		/*public void moveTwoTotesAuto(){
		switch (step){
		case 1: 
		while (autoTimer.get() < .5){
		driveValue[0] = .5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 2: 
		while (autoEye.get()){
		driveValue[1] = .5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 4:
		liftElevator(1);
		while (!notColor){
		driveValue[0]= -.5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 5:
		solenoid.set(false);
		liftElevator(0);
		solenoid.set(true);
		nextAutoStep();
		break;

		case 6:
		gryo.reset();
		while (Math.abs(gyro.getAngle) < 90)){
		driveValue[2] = .25;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 7: 

		driveValue[1] = .5;
		nextAutoStep();
		break;
		}
		}



		public void moveThreeTotesAuto(){
		switch (step){
		case 1: 
		while (autoTimer.get() < .5){
		driveValue[0] = .5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 2: 
		while (autoEye.get()){
		driveValue[1] = .5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 3:
		liftElevator(1);
		while (!notColor){
		driveValue[0]= -.5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 4:
		solenoid.set(false);
		liftElevator(0);
		solenoid.set(true);
		nextAutoStep();
		break;

		case 5: 
		while (autoTimer.get() < .5){
		driveValue[0] = .5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 6: 
		while (autoEye.get()){
		driveValue[1] = .5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 7:
		liftElevator(1);
		while (!notColor){
		driveValue[0]= -.5;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 8:
		arms.set(false);
		liftElevator(0);
		solenoid.set(true);
		nextAutoStep();
		break;

		case 9:
		gryo.reset();
		while (Math.abs(gyro.getAngle) < 90)){
		driveValue[2] = .25;
		}
		resetDriveValues();
		nextAutoStep();
		break;

		case 10: 

		driveValue[1] = .5;
		break;
		}
		} */

		    
		    
		
}
