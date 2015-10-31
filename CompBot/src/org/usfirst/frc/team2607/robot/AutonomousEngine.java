package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.PrintWriter;
import java.lang.reflect.Array;
import java.util.Scanner;
import java.util.Vector;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousEngine implements Runnable {
	Timer autoTimer;
	Robot theBot;
	robovikingMotionProfiler motion;

	int step;
	int mode;


	public AutonomousEngine(Robot robot){
		theBot = robot;
		autoTimer = new Timer();
		step = 0;
		mode = 0;
		theBot.BackL.setGearPID(false);
		theBot.FrontL.setGearPID(false);
		theBot.BackR.setGearPID(false);
		theBot.FrontR.setGearPID(false);
		motion = new robovikingMotionProfiler(theBot.robotDrive);
	}

	public void displayMode() {
		SmartDashboard.putNumber("autoMode", mode);
		switch(mode) {
			case 0:
				SmartDashboard.putString("autonMode", "Auton: NONE");
				break;
			case 1:
				SmartDashboard.putString("autonMode", "Auton: 1 Can Clockwise");
				break;
			case 2:
				SmartDashboard.putString("autonMode", "Auton: 1 Can CounterClockwise");
				break;
			case 3:
				SmartDashboard.putString("autonMode", "Auton: 1 Can and 1 Tote");
				break;
			case 4:
				SmartDashboard.putString("autonMode", "Auton: Full Three Tote");
				break;
			case 5:
				SmartDashboard.putString("autonMode", "Auton: 1 Tote");
				break;
			case 6:
				SmartDashboard.putString("autonMode", "Auton: 1-Can Backwards");
				break;
			case 7:
				SmartDashboard.putString("autonMode", "Auton: Just Drive Forward");
				break;
			case 8:
				SmartDashboard.putString("autonMode", "Auton: Drive Straight Three Tote");
				break;
			case 9:
				SmartDashboard.putString("autonMode", "Auton: Strafe Three Tote (NONFUNCTIONAL)");
				break;
			case 10:
				SmartDashboard.putString("autonMode", "Auton: Test 10");
				break;
			case 11:
				SmartDashboard.putString("autonMode", "Auton: Test 11");
				break;
			case 12:
				SmartDashboard.putString("autonMode", "Auton: Test 12");
				break;
			case 13:
				SmartDashboard.putString("autonMode", "Auton: Test 13");
				break;
			case 14:
				SmartDashboard.putString("autonMode", "Auton: Setting up for teleop clockwise rotation");
				break;
			case 15:
				SmartDashboard.putString("autonMode", "Auton: Setting up for Can Burglar");
				break;	
			case 16: SmartDashboard.putString("autonMode", "Auton: Setting up for teleop long clockwise rotation");
				break;
			
			default:
				SmartDashboard.putString("autonMode", "UNKNOWN!!");
				break;
		}	
	
	}
	
	public void saveMode() {
		try {
			PrintWriter p = new PrintWriter(new File("/home/lvuser/autoMode.txt"));
			p.printf("%d", mode);
			p.flush();
			p.close();
		} catch (Exception e) {
			
		}
	}
	
	public void selectMode() {
		if (++mode > 16) mode = 0;
		saveMode();
		displayMode();
	}

	public void loadSavedMode() {
		try {
			FileInputStream fin = new FileInputStream("/home/lvuser/autoMode.txt");
			Scanner s = new Scanner(fin);
			if (s.hasNextInt()) mode = s.nextInt();
			else mode = 0;
			fin.close();
		} catch (Exception e) {
			mode = 0;
		}
		displayMode();
	}
	
	//True 3-tote auto
	private void strafeThreeTote() {
		Vector<Double> strafeRight = new Vector<Double>();
		strafeRight.add(.35);
		strafeRight.add(0.0);

		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.5);

		Vector<Double> strafeLeft = new Vector<Double>();
		strafeLeft.add(-.35);
		strafeLeft.add(-.08);

		theBot.robotDrive.resetDistance();
		try {
			// close the hooks
			theBot.motaVator.arms.set(true);
			Thread.sleep(330);
			// set elevator to carrying position
			theBot.motaVator.goToHeight(-18.5);
			// strafe right
			Thread.sleep(100);
			motion.setFtB(Constants.ftbCorrectionOneTote);
			motion.driveUntilDistance(31,  strafeRight, false);
			// drive forward to next tote
			motion.driveUntilDistance(87,  forward, false);
			// Stop so mecanum wheels can accelerate together
			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
			Thread.sleep(600);
			// strafe left
			motion.driveUntilDistance(36,  strafeLeft, false);

			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

			theBot.motaVator.lowerManual();
			while(theBot.motaVator.enc.getDistance() < -12) Thread.sleep(2);
			theBot.motaVator.equilibrium();

			theBot.motaVator.arms.set(false);
			Thread.sleep(100);

			theBot.motaVator.goToHeight(0);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

			theBot.motaVator.arms.set(true);
			Thread.sleep(700);

			theBot.motaVator.goToHeight(-18.5);

			// strafe right
			motion.setFtB(Constants.ftbCorrectionTwoTote);
			motion.driveUntilDistance(36,  strafeRight, false);

			motion.driveUntilDistance(90,  forward, false);

			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
			Thread.sleep(600);
			// strafe left
			motion.driveUntilDistance(38,  strafeLeft, false);

			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

			/*motion.driveUntilDistance(82.5,  forward, false);
			// Stop so mecanum wheels can accelerate together
			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
			Thread.sleep(600);
			// strafe left
			motion.driveUntilDistance(40,  strafeLeft, false);

			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

			theBot.motaVator.lowerManual();
			while(theBot.motaVator.enc.getDistance() < -12) Thread.sleep(2);
			theBot.motaVator.equilibrium();

			theBot.motaVator.arms.set(false);
			Thread.sleep(100);

			theBot.motaVator.goToHeight(-.5);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

			theBot.motaVator.arms.set(true);
			Thread.sleep(500);

			theBot.motaVator.goToHeight(-18.5);

			// strafe right
			motion.driveUntilDistance(30,  strafeRight, false);

			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
			 *
			 *
			 */
		} catch (Exception e) {}
	}

	//Drive straight 3 tote auto
	private void driveStraightThreeTote() {
		Vector<Double> strafeRight = new Vector<Double>();
		strafeRight.add(.6);
		strafeRight.add(.2);

		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.6);

		Vector<Double> fast = new Vector<Double>();
		fast.add(0.0);
		fast.add(-.9);

		Vector<Double> strafeLeft = new Vector<Double>();
		strafeLeft.add(-.35);
		strafeLeft.add(-.08);

		Vector<Double> back = new Vector<Double>();
		back.add(0.0);
		back.add(.1);

		theBot.robotDrive.resetDistance();
		try {
			// close the hooks
			theBot.motaVator.arms.set(true);
			Thread.sleep(2000);
			// set elevator to carrying position
			theBot.motaVator.goToHeight(-18.5);

			// drive forward to next tote
			motion.driveUntilDistance(78.4,  forward, false);
			// Stop so mecanum wheels can accelerate together
			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

			theBot.motaVator.lowerManual();
			while(theBot.motaVator.enc.getDistance() < -13) Thread.sleep(2);
			theBot.motaVator.equilibrium();

			theBot.motaVator.arms.set(false);
			Thread.sleep(100);

			
			theBot.motaVator.goToHeight(-1);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

			theBot.motaVator.arms.set(true);
			Thread.sleep(600);

			// set elevator to carrying position
			theBot.motaVator.goToHeight(-18.5);
			// drive forward to next tote
			motion.driveUntilDistance(78.4,  forward, false);
			// Stop so mecanum wheels can accelerate together
			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

			theBot.motaVator.lowerManual();
			while(theBot.motaVator.enc.getDistance() < -13) Thread.sleep(2);
			theBot.motaVator.equilibrium();

			theBot.motaVator.arms.set(false);
			Thread.sleep(100);

			theBot.motaVator.goToHeight(-1);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

			theBot.motaVator.arms.set(true);
			Thread.sleep(600);

			motion.setFtB(Constants.ftbCorrectionTwoTote);
			motion.driveUntilDistance(40,  strafeRight, false);

			System.out.println("Rotating!");
			motion.rotateUntilDegree(90, false);

			System.out.println("Driving!");
			motion.driveUntilDistance(87, fast, false);

			theBot.motaVator.arms.set(false);

			motion.driveUntilDistance(10, back, false);
		} catch (Exception e){

		}
	}

	//Stacks a Recycling container on a tote, then rotates and drives to auto zone
	public void oneCanOneTote(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.35);

		theBot.robotDrive.resetDistance();

		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(500);

		theBot.motaVator.goToHeight(-18);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

		motion.driveUntilDistance(17, forward, false);

		theBot.motaVator.goToHeight(-15);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

		theBot.motaVator.arms.set(false);

		theBot.motaVator.goToHeight(-3);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

		theBot.motaVator.arms.set(true);
		Thread.sleep(500);

		motion.rotateUntilDegree(-90, false);
		Thread.sleep(300);

		motion.driveUntilDistance(115, forward, false);

		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.goToHeight(-1);

		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

		theBot.motaVator.arms.set(false);

		} catch (InterruptedException e) {

		}

	}

	// Just Drives Forward
	public void driveForward(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.3);

		theBot.robotDrive.resetDistance();

		motion.driveUntilDistance(75, forward, false);

		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

	}

	//Grabs recycling container, rotates 90 degrees, and drives to auto zone
	public void oneCanCounterclockwise(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.4);

		theBot.robotDrive.resetDistance();

		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(500);

		theBot.motaVator.goToHeight(-8);
		Thread.sleep(300);
//		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);

		motion.rotateUntilDegree(-90, false);
		Thread.sleep(300);

		motion.driveUntilDistance(115, forward, false);

		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.goToHeight(-1);

//		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		theBot.motaVator.arms.set(false);


		} catch (InterruptedException e) {

		}


	}

	//One Tote Auto
	public void oneTote(){
//		Vector<Double> forward = new Vector<Double>();
//		forward.add(0.0);
//		forward.add(-.3);
//
//		theBot.robotDrive.resetDistance();
//
//		try {
//		theBot.motaVator.arms.set(true);
//
//		motion.rotateUntilDegree(90, false);
//
//		Thread.sleep(300);
//
//		motion.driveUntilDistance(150, forward, false);
//
//		} catch (InterruptedException e) {
//		}

		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.4);

		theBot.robotDrive.resetDistance();

		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(500);

		theBot.motaVator.goToHeight(-18);
//		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		
		motion.rotateUntilDegree(90, false);
		Thread.sleep(300);

		motion.driveUntilDistance(115, forward, false);

		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.goToHeight(-1);

		//while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		theBot.motaVator.arms.set(false);


		} catch (InterruptedException e) {

		}


	}
	
	//grabs a recycling container and moves backwards
	public void oneCanDriveBackwards(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.4);
		
		Vector<Double> back = new Vector<Double>();
		back.add(0.0);
		back.add(.4);
		
		

		theBot.robotDrive.resetDistance();

		try {
		theBot.motaVator.goToHeight(-8.0);
//		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		
		motion.driveUntilDistance(21, forward, false);
			
		theBot.motaVator.arms.set(true);
		Thread.sleep(500);

		theBot.motaVator.goToHeight(-12);
		Thread.sleep(400);

		motion.driveUntilDistance(100, back, false);

		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.goToHeight(-7);

		//while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		
		theBot.motaVator.arms.set(false);
		
		} catch (InterruptedException e) {

		}

		
	}
	
	//Grabs recycling container, rotates other way 90 degrees, and drives to auto zone
	public void oneCanClockwise(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.4);

		theBot.robotDrive.resetDistance();

		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(500);

		theBot.motaVator.goToHeight(-18);
		//while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		
		motion.rotateUntilDegree(90, false);
		Thread.sleep(300);

		motion.driveUntilDistance(115, forward, false);

		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.goToHeight(-1);

		//while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		Thread.sleep(300);
		
		theBot.motaVator.arms.set(false);


		} catch (InterruptedException e) {

		}


	}
	
	//3 Tote auton for reals this time, no joke. AKA God Mode
	public void tryTwoThreeTote(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.6);
		
		Vector<Double> strafeRight = new Vector<Double>();
		strafeRight.add(.35);
		strafeRight.add(0.0);
		
		Vector<Double> strafeRightFast = new Vector<Double>();
		strafeRight.add(.75);
		strafeRight.add(0.0);
		
		Vector<Double> back = new Vector<Double>();
		forward.add(0.0);
		forward.add(.4);
		
		theBot.robotDrive.resetDistance();
		
		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(300);
		
		theBot.motaVator.goToHeight(-6);
		
		motion.rotateUntilDegree(40, false);
		Thread.sleep(300);
		
		motion.driveUntilDistance(10, forward, false);
		Thread.sleep(300);
		
		motion.rotateUntilDegree(-40, false);
		
		motion.driveUntilDistance(8, strafeRight, false);
		Thread.sleep(500);
		
		
		motion.driveUntilDistance(87,  forward, false);
		theBot.motaVator.goToHeight(-14);
		
		Thread.sleep(500);
		
		theBot.motaVator.goToHeight(-10);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		Thread.sleep(300);
		
		theBot.motaVator.arms.set(false);
		
		theBot.motaVator.goToHeight(-1);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		theBot.motaVator.arms.set(true);
		
		Thread.sleep(300);
		
		theBot.motaVator.goToHeight(-6);
		
		motion.rotateUntilDegree(40, false);
		Thread.sleep(300);
		
		motion.driveUntilDistance(10, forward, false);
		Thread.sleep(300);
		
		motion.rotateUntilDegree(-40, false);
		
		motion.driveUntilDistance(8, strafeRight, false);
		Thread.sleep(500);
		
		
		motion.driveUntilDistance(87,  forward, false);
		theBot.motaVator.goToHeight(-14);
		
		Thread.sleep(500);
		
		theBot.motaVator.goToHeight(-10);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		Thread.sleep(300);
		
		theBot.motaVator.arms.set(false);
		
		theBot.motaVator.goToHeight(-1);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		theBot.motaVator.arms.set(true);
		
		Thread.sleep(300);
		
		motion.driveUntilDistance(100,  strafeRightFast, false);
		
		theBot.motaVator.goToHeight(-1);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		theBot.motaVator.arms.set(false);
		
		motion.driveUntilDistance(8, back, false);
		
		
		
		} catch (InterruptedException e){
			
		}
	}
	
	//strafes at -45 degrees until reaching 48 inches
	public void testAutonMode(){
		theBot.robotDrive.resetDistance();
		motion.driveUntilDistancePulse(48, .8, -45, false);
		
	}
	
	
	//3 tote auton buy pushing totes, 30 degree angles
	public void tryThreeThreeTote(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.6);
		
		Vector<Double> fastforward = new Vector<Double>();
		fastforward.add(0.0);
		fastforward.add(-.8);
		

		
		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(300);
		motion.rotateUntilDegree(-30, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);		
		theBot.motaVator.goToCarryingPos();
			
		motion.driveUntilDistance(17, forward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		motion.rotateUntilDegree(60, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		motion.driveUntilDistance(17, fastforward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		Thread.sleep(200);
		motion.rotateUntilDegree(-30, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		Thread.sleep(200);
		
		theBot.motaVator.goToHeight(-18);
		Thread.sleep(350);
		motion.driveUntilDistance(55.5, forward, false);  
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		theBot.motaVator.lowerManual();
		while(theBot.motaVator.enc.getDistance() < -12) Thread.sleep(2);
		theBot.motaVator.equilibrium();
		theBot.motaVator.arms.set(false);
		Thread.sleep(200);
		motion.setFtB(Constants.ftbCorrectionTwoTote);
		theBot.motaVator.goToHeight(-1);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		theBot.motaVator.arms.set(true);
		Thread.sleep(200);
		motion.rotateUntilDegree(-30, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);		
		theBot.motaVator.goToCarryingPos();
			
		motion.driveUntilDistance(17, forward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		motion.rotateUntilDegree(60, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		motion.driveUntilDistance(17, fastforward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		Thread.sleep(200);
		motion.rotateUntilDegree(-30, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		Thread.sleep(200);
		
		theBot.motaVator.goToHeight(-18);
		Thread.sleep(350);
		motion.driveUntilDistance(58.5, forward, false);  
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.lowerManual();
		while(theBot.motaVator.enc.getDistance() < -12) Thread.sleep(2);
		theBot.motaVator.equilibrium();
		theBot.motaVator.arms.set(false);
		Thread.sleep(200);
		theBot.motaVator.goToHeight(-1);
		while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
		
		theBot.motaVator.arms.set(true);
		Thread.sleep(200);
		theBot.motaVator.goToCarryingPos();
		
		motion.dsAcceptableRange=(15);
		motion.rotateUntilDegree(80, false);
		motion.dsAcceptableRange=(3);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
    	theBot.gearShiftSolenoid.set(true);    	    	
    	theBot.FrontL.setGearPID(true);
    	theBot.FrontR.setGearPID(true);
    	theBot.BackL.setGearPID(true);
    	theBot.BackR.setGearPID(true);
    	
		theBot.motaVator.goToHeight(0.0);
		motion.driveUntilDistance(75, forward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);


		theBot.motaVator.arms.set(false);
		
    	theBot.gearShiftSolenoid.set(false);    	    	
    	theBot.FrontL.setGearPID(false);
    	theBot.FrontR.setGearPID(false);
    	theBot.BackL.setGearPID(false);
    	theBot.BackR.setGearPID(false);
		
		
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}	
		
	}
	
	
	public void anotherAutonTest(){
		motion.driveUntilTargetRotation(.6, -90.0, 90.0, -.3, false);
	}
	
	public void workingThreeTote(){
		int vatorTimeOut = 0;
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.6);
		
		Vector<Double> fastforward = new Vector<Double>();
		fastforward.add(0.0);
		fastforward.add(-.8);
		
		Vector<Double> fasterforward = new Vector<Double>();
		fasterforward.add(0.0);
		fasterforward.add(-.92);

		
		try {
		theBot.motaVator.arms.set(true);
		Thread.sleep(300);
		motion.rotateUntilDegree(-20, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);		
		theBot.motaVator.goToCarryingPos();
			
		motion.driveUntilDistance(13, forward, false); //10
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		motion.rotateUntilDegree(40, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		motion.driveUntilDistance(8.5, forward, false); //16
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		Thread.sleep(200);
		motion.rotateUntilDegree(-20, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		Thread.sleep(200);
		
		theBot.motaVator.goToHeight(-18);
		Thread.sleep(350);
		motion.driveUntilDistance(59.5, forward, false);  //59
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		theBot.motaVator.lowerManual();
		vatorTimeOut = 0;
		while(theBot.motaVator.enc.getDistance() < -12) { 
			if (++vatorTimeOut > 750) {
				theBot.motaVator.equilibrium();
				return;
			}
			Thread.sleep(2);
		}
		theBot.motaVator.equilibrium();
		theBot.motaVator.arms.set(false);
		Thread.sleep(200);
		motion.setFtB(Constants.ftbCorrectionTwoTote);
		theBot.motaVator.goToHeight(-1);
		vatorTimeOut = 0;
		while(!theBot.motaVator.pid.onTarget() ) { 
			if (++vatorTimeOut > 750) {
				theBot.motaVator.equilibrium();
				return;
			}
			Thread.sleep(2);
		}
			
		theBot.motaVator.arms.set(true);
		Thread.sleep(200);
		motion.rotateUntilDegree(-20, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);		
		theBot.motaVator.goToCarryingPos();
			
		motion.driveUntilDistance(13, forward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		motion.rotateUntilDegree(40, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		motion.driveUntilDistance(8.5, forward, false); // 16
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		Thread.sleep(200);
		motion.rotateUntilDegree(-20, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
		
		Thread.sleep(200);
		
		theBot.motaVator.goToHeight(-18);
		Thread.sleep(350);
		motion.driveUntilDistance(59.5, forward, false);  //61.5
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);

		theBot.motaVator.lowerManual();
		vatorTimeOut = 0;
		while(theBot.motaVator.enc.getDistance() < -12) { 
			if (++vatorTimeOut > 750) {
				theBot.motaVator.equilibrium();
				return;
			}
			Thread.sleep(2);
		}
		theBot.motaVator.equilibrium();
		theBot.motaVator.arms.set(false);
		Thread.sleep(200);
		theBot.motaVator.goToHeight(-1);
		vatorTimeOut = 0;
		while(!theBot.motaVator.pid.onTarget() ) { 
			if (++vatorTimeOut > 750) {
				theBot.motaVator.equilibrium();
				return;
			}
			Thread.sleep(2);
		}
		
		theBot.motaVator.arms.set(true);
		Thread.sleep(200);
		theBot.motaVator.goToCarryingPos();
		
		motion.dsAcceptableRange=(15);
		motion.rotateUntilDegree(80, false);
		motion.dsAcceptableRange=(3);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
	
    	theBot.gearShiftSolenoid.set(true);    	    	
    	theBot.FrontL.setGearPID(true);
    	theBot.FrontR.setGearPID(true);
    	theBot.BackL.setGearPID(true);
    	theBot.BackR.setGearPID(true);
    	
		theBot.motaVator.goToHeight(0.0);
		motion.driveUntilDistance(70, fastforward, false);
		theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);


		theBot.motaVator.arms.set(false);
		
    	theBot.gearShiftSolenoid.set(false);    	    	
    	theBot.FrontL.setGearPID(false);
    	theBot.FrontR.setGearPID(false);
    	theBot.BackL.setGearPID(false);
    	theBot.BackR.setGearPID(false);
		
		
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}	
		
	}
	
	private void teleopSetUpClockWise(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.4);
		
		Vector<Double> backwards = new Vector<Double>();
		backwards.add(0.0);
		backwards.add(.4);
		
		theBot.robotDrive.resetDistance();
		
		try{
			theBot.motaVator.arms.set(true);
			Thread.sleep(500);
			
			theBot.motaVator.goToHeight(-5);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
			
			motion.rotateUntilDegree(45, false);
			Thread.sleep(300);
			
			motion.driveUntilDistance(73, forward, false);
			
			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
			
			theBot.motaVator.goToHeight(-1);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
			
			theBot.motaVator.arms.set(false);
			
			motion.driveUntilDistance(25, backwards, false);
			
		} catch(InterruptedException e){
			
			
			
		}
		
	}
	
	private void canBurgle(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-1.0);
		
		Vector<Double> backwards = new Vector<Double>();
		backwards.add(0.0);
		backwards.add(1.0);
		
		try{
			
			//theBot.robotDrive.correctedMecanumDrive(0,0.25,0,0,0);
			theBot.canBurglar.set(true);
			Thread.sleep(800);
			
			theBot.robotDrive.correctedMecanumDrive(0,0,0,0,0);
	    	theBot.gearShiftSolenoid.set(true);    	    	
	    	theBot.FrontL.setGearPID(true);
	    	theBot.FrontR.setGearPID(true);
	    	theBot.BackL.setGearPID(true);
	    	theBot.BackR.setGearPID(true);
	    	
	    	//motion.driveUntilDistance(76, forward, false);
	    	
	    	theBot.robotDrive.correctedMecanumDrive(0, -1, -.48, 0, 0);
	    	
	    	Thread.sleep(1150);
	    	
	    	theBot.robotDrive.correctedMecanumDrive(0, 0,0,0,0);
	    	theBot.gearShiftSolenoid.set(false);    	    	
	    	theBot.FrontL.setGearPID(false);
	    	theBot.FrontR.setGearPID(false);
	    	theBot.BackL.setGearPID(false);
	    	theBot.BackR.setGearPID(false);
	    	
	    	theBot.canBurglar.set(false);
			
//		theBot.robotDrive.resetDistance();
//		
//		try{
//			theBot.motaVator.arms.set(true);
//			Thread.sleep(500);
//			
//			theBot.motaVator.goToHeight(-5);
//			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
//			
//			motion.rotateUntilDegree(-45, false);
//			Thread.sleep(300);
//			
//			motion.driveUntilDistance(73, forward, false);
//			
//			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
//			
//			theBot.motaVator.goToHeight(-1);
//			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
//			
//			theBot.motaVator.arms.set(false);
//			
//			motion.driveUntilDistance(25, backwards, false);
			
		} catch(InterruptedException e){
			
			
			
		}
		
	}
	
	private void teleopSetUpLongClockWise(){
		Vector<Double> forward = new Vector<Double>();
		forward.add(0.0);
		forward.add(-.4);
		
		Vector<Double> backwards = new Vector<Double>();
		backwards.add(0.0);
		backwards.add(.4);
		
		theBot.robotDrive.resetDistance();
		
		try{
			theBot.motaVator.arms.set(true);
			Thread.sleep(500);
			
			theBot.motaVator.goToHeight(-5);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
			
			motion.rotateUntilDegree(135, false);
			Thread.sleep(300);
			
			motion.driveUntilDistance(73, forward, false);
			
			theBot.robotDrive.correctedMecanumDrive(0, 0, 0, 0, 0);
			
			theBot.motaVator.goToHeight(-1);
			while(!theBot.motaVator.pid.onTarget()) Thread.sleep(2);
			
			theBot.motaVator.arms.set(false);
			
			motion.driveUntilDistance(25, backwards, false);
			
		} catch(InterruptedException e){
			
			
			
		}
		
	}


	@Override
	public void run() {
		// called by Thread.start();
		System.out.println("Auto Thread Start");
			switch (mode) {
			case 0:
				// turn off outputs
				break;
			case 1:
				System.out.println("Running Auto 1");
				oneCanClockwise();
				mode = 0;
				break;
			case 2:
				System.out.println("Running Auto 2");
				oneCanCounterclockwise();
				mode = 0;
				break;

			case 3:
				System.out.println("Running Auto 3");
				oneCanOneTote();
				mode = 0;
				break;

			case 4:
				System.out.println("Running Auto 4");
				workingThreeTote();
				mode = 0;
				break;

			case 5:
				System.out.println("Running Auto 5");
				oneTote();
				mode = 0;
				break;

			case 6:
				System.out.println("Running Auto 6");
				oneCanDriveBackwards();
				mode = 0;
				break;
				
			case 7:
				System.out.println("Running Auto 7");
				driveForward();
				mode = 0;
				break;
			case 8:
				System.out.println("Running Auto 8");
				driveStraightThreeTote();	// only exits when done, or interrupted
				mode = 0;
				break;
				
			case 9:
				System.out.println("Running Auto 9");
				strafeThreeTote();	// only exits when done, or interrupted
				mode = 0;
				break;
				
			case 10:
				System.out.println("Running Auto 10");
				testAutonMode();
				mode = 0;
				break;
				
			case 11: 
				System.out.println("Running Auto 11");
				tryThreeThreeTote();
				mode = 0;
				break;
				
			case 12:
				System.out.println("Running Auto 12");
				anotherAutonTest();
				mode = 0;
				break;
			
			case 13:
				System.out.println("Running Auto 13");
				tryTwoThreeTote();
				mode = 0;
				break;
			case 14:
				System.out.println("Running Auto 14");
				teleopSetUpClockWise();
				mode = 0;
				break;
			case 15:
				System.out.println("Running Auto 15");
				canBurgle();
				mode = 0;
				break;
			case 16:
				System.out.println("Runnint Auto 16");
				teleopSetUpLongClockWise();
				mode = 0;
				break;
//				case 0:
//					// turn off outputs
//					break;
//				case 1:
//					System.out.println("Running Auto 1");
//					strafeThreeTote();	// only exits when done, or interrupted
//					mode = 0;
//					break;
//				case 2:
//					System.out.println("Running Auto 2");
//					driveStraightThreeTote();	// only exits when done, or interrupted
//					mode = 0;
//					break;
//
//				case 3:
//					System.out.println("Running Auto 3");
//					oneCanOneTote();
//					mode = 0;
//					break;
//
//				case 4:
//					System.out.println("Running Auto 4");
//					driveForward();
//					mode = 0;
//					break;
//
//				case 5:
//					System.out.println("Running Auto 5");
//					oneCanCounterclockwise();
//					mode = 0;
//					break;
//
//				case 6:
//					System.out.println("Running Auto 6");
//					oneTote();
//					mode = 0;
//					break;
//					
//				case 7:
//					System.out.println("Running Auto 7");
//					oneCanDriveBackwards();
//					mode = 0;
//					break;
//				case 8:
//					System.out.println("Running Auto 8");
//					oneCanClockwise();
//					mode = 0;
//					break;
//					
//				case 9:
//					System.out.println("Running Auto 9");
//					tryTwoThreeTote();
//					mode = 0;
//					break;
//					
//				case 10:
//					System.out.println("Running Auto 10");
//					testAutonMode();
//					mode = 0;
//					break;
//					
//				case 11: 
//					System.out.println("Running Auto 11");
//					tryThreeThreeTote();
//					mode = 0;
//					break;
//					
//				case 12:
//					System.out.println("Running Auto 12");
//					anotherAutonTest();
//					mode = 0;
//				
//				case 13:
//					System.out.println("Running Auto 13");
//					workingThreeTote();
//					mode = 0;

				default:
			}
		System.out.println("Exiting Auto");

	}



}
