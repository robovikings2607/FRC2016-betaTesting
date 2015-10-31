
package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	WheelRPMController FrontL;
	WheelRPMController FrontR;
	WheelRPMController BackL;
	WheelRPMController BackR;
	
	Logger logger;
	Thread loggerThread = null;
	boolean navXInitialized = false;
	Elevator motaVator; // this is the elevator....
	AutonomousEngine auto;
	Thread autoThread = null;
	
	Ultrasonic ultraSide, ultraFront;
	
	IMUAdvanced navx;
	SerialPort comPort;
	Solenoid gearShiftSolenoid;
	robovikingMecanumDrive robotDrive;
	robovikingStick xboxSupremeController, xboxMinor;
	SmartDashboard smartDash;
	boolean inAuto = false, inTeleop = false;
	double x, y, z;
	double[] driveValue = new double[3];
	double[] deadZones = {0.15, 0.15, 0.15};
	double tempAngle = 0;
	boolean extraLowGear = false;
	double gearCoefficient =1.5;
	Solenoid canBurglar;
	GyroPIDController gyroPID;
	double targetGyroPIDAngle = 0;  
	boolean maintenanceModeWasRun = false;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	int calCounter = 0;
    	motaVator = new Elevator();
    //	new Thread(motaVator).start();
    	
    	xboxSupremeController = new robovikingStick(0);
    	xboxMinor = new robovikingStick(1);
    	FrontL = new WheelRPMController("FrontLeft",0,true);
    	FrontR = new WheelRPMController("FrontRight",1,true);
    	BackL = new WheelRPMController("BackLeft", 2,true);
    	BackR = new WheelRPMController("BackRight", 3,true);

    	gearShiftSolenoid = new Solenoid(1, Constants.gearShiftChannel);
    	canBurglar = new Solenoid(1, Constants.breaksChannel);
    	
    	
    	ultraFront = new Ultrasonic(Constants.ultraFrontPing, Constants.ultraFrontEcho);
    	ultraSide = new Ultrasonic(Constants.ultraSidePing,Constants.ultraSideEcho);
    	ultraFront.setAutomaticMode(true);
    	ultraSide.setAutomaticMode(true);
    	ultraFront.setEnabled(true);
    	ultraSide.setEnabled(true);
 
    	logger = new Logger(this);
    	logger.start();
    	inAuto = false;
    	inTeleop = false;
    	FrontL.enable();
    	BackL.enable();
    	FrontR.enable();
    	BackR.enable();
    	try {
            comPort = new SerialPort(57600, SerialPort.Port.kMXP);
                    
                    // You can add a second parameter to modify the 
                    // update rate (in hz) from 4 to 100.  The default is 100.
                    // If you need to minimize CPU load, you can set it to a
                    // lower value, as shown here, depending upon your needs.
                    
                    // You can also use the IMUAdvanced class for advanced
                    // features.
                    
                    byte update_rate_hz = 50;
                    //imu = new IMU(serial_port,update_rate_hz);
                    navx = new IMUAdvanced(comPort, update_rate_hz);
                	while (navx.isCalibrating() && ++calCounter < 6000) {
                		Thread.sleep(5);
                	}
                	if (calCounter < 6000) { 
                		navXInitialized = true;
                	} else {
                		navXInitialized = false;
                	}
                	navx.zeroYaw();
            } catch( Exception ex ) {
            	navXInitialized = false;
            	ex.printStackTrace();
            }
    	GyroArduinoUpdater ardu = new GyroArduinoUpdater(navx, navXInitialized);
    	gyroPID = new GyroPIDController(0.014, .000, 0.006, 0.04, navx);

    	robotDrive = new robovikingMecanumDrive(FrontL, BackL, FrontR, BackR, navx);
    	robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
    	robotDrive.setInvertedMotor(MotorType.kRearLeft, true);    
    	robotDrive.setSafetyEnabled(false);
    	smartDash = new SmartDashboard();
    	
    	auto = new AutonomousEngine(this);
    	auto.loadSavedMode();
    }

    public void disabledInit() {
    	
    	robotDrive.correctedMecanumDrive(0,0,0, 0.0, 0);
    	if (navXInitialized) {
    		smartDash.putString("navX Calibration", "GOOD");
    	} else {
    		smartDash.putString("navX Calibration", "BAD");
    	}
    }

    public void disabledPeriodic(){
    	if (xboxSupremeController.getButtonPressedOneShot(8)){
    		// increment auto mode
    		auto.selectMode();
    	}
    	smartDash.putBoolean("topSwtich", motaVator.topSwitch.get());
    	if (autoThread != null) { 
    		autoThread.interrupt();
    	}
    }
    
    /**
     * This function is called periodically during autonomous
     */
    
    public void autonomousInit(){
    	if (maintenanceModeWasRun) return;
    	autoThread = new Thread(auto);
    	if (navXInitialized) {
    		navx.zeroYaw();
    		autoThread.start();
    		if (logger != null) {
    			logger.enableLogging(true);
    		}
    	} else {
    		SmartDashboard.putString("autonMode", "SKIPPED");
    	}
    }
    
    public void autonomousPeriodic() {
    	if (maintenanceModeWasRun) return;
    	inAuto = true;
    	inTeleop = false;
    }

    
    
    boolean elevatorHeightChangedOneShot;  
    public void teleopInit(){
    	//loggerThread = new Thread(logger);
    	//loggerThread.start();
    	elevatorHeightChangedOneShot = false;
    	gearCoefficient = 1.5;
    }
    

    /**
     * This function is called periodically during operator control
     */
 
    public void teleopPeriodic() {
    	inAuto = false;
    	inTeleop = true;
    	if (maintenanceModeWasRun) return;
    	//logger.enableLogging(xboxSupremeController.getToggleButton(7));
    	//BackL.enableLogging(xboxSupremeController.getToggleButton(7));
    	
    	//Shifting Management - Button 9
    	gearShiftSolenoid.set(xboxSupremeController.getToggleButton(9));    	    	
    	FrontL.setGearPID(xboxSupremeController.getToggleButton(9));
    	FrontR.setGearPID(xboxSupremeController.getToggleButton(9));
    	BackL.setGearPID(xboxSupremeController.getToggleButton(9));
    	BackR.setGearPID(xboxSupremeController.getToggleButton(9));
    	
    	//Some dead-zone stuff that nobody understands anymore, but hey, it works
    	if(xboxSupremeController.getToggleButton(10)){
	    	driveValue[0] = xboxSupremeController.getX() * .65 * .5;
	    	driveValue[1] = xboxSupremeController.getY() * .65 * .5;
	    	driveValue[2] = xboxSupremeController.getRawAxis(4)/2 * .5;
    	} else {
	    	driveValue[0] = xboxSupremeController.getX() * .65;
	    	driveValue[1] = xboxSupremeController.getY() * .65;
	    	driveValue[2] = xboxSupremeController.getRawAxis(4)/2;
    	}
    	
//    	if (xboxSupremeController.getButtonPressedOneShot(10)){
//    		extraLowGear = !extraLowGear;
//    		gearCoefficient = extraLowGear ? 0.5:1.5;
//    		System.out.println("Changed " + extraLowGear);
//    	}
    	
    	for (int i = 0; i <= 2; i++) {
    		if (Math.abs(driveValue[i]) <= deadZones[i]) {
    			driveValue[i] = 0;
    		}
    		if (driveValue[i] > deadZones[i] && driveValue[i] <= deadZones[i] * 2) {
    			driveValue[i] = (driveValue[i] - .15) * gearCoefficient;
    		}
    		if (driveValue[i] < -deadZones[i] && driveValue[i] >= -2 * deadZones[i]) {
    			driveValue[i] = (driveValue[i] + .15) * gearCoefficient;
    		}
	    	}
    	
    	if(xboxSupremeController.getButtonPressedOneShot(3)){
    		gyroPID.enable();
    		gyroPID.setSetpoint(targetGyroPIDAngle);
    	}
    	if(xboxSupremeController.getRawButton(3)){
    		driveValue[2] = gyroPID.get();
    	}
    	if(xboxSupremeController.getButtonReleasedOneShot(3)){
    		gyroPID.disable();
    	}
    	
    	//Manual Elevator Control - Button 1 & 4
	    if((xboxSupremeController.getRawButton(1) || xboxMinor.getRawButton(1))){
    		motaVator.lowerManual();
    	} else if((xboxSupremeController.getRawButton(4) || xboxMinor.getRawButton(4))){
    		motaVator.raiseManual();
    	} 
    	
	    //Hold position for manual control
	    if (xboxSupremeController.getButtonReleasedOneShot(1) || xboxMinor.getButtonReleasedOneShot(1) || 
	    	xboxSupremeController.getButtonReleasedOneShot(4) || xboxMinor.getButtonReleasedOneShot(4)) {
    			motaVator.holdCurrentPosition();
    	}
    
    	//Actuate arms - Button 2
    	if(xboxSupremeController.getButtonPressedOneShot(2) || (xboxMinor.getButtonPressedOneShot(2))){
    	motaVator.grab();
    	}
    	
    	//CanBurgle - CoPilot Button 3
    	canBurglar.set(xboxMinor.getToggleButton(3));
    	
    	//Move to various tote heights - D-Pad
    	switch (xboxSupremeController.getPOV(0)){
    		case 0:
    			motaVator.goToLevel(1);
    			break;
    		case 90:
    			motaVator.goToLevel(2);
    			break;
    		case 180:
    			motaVator.goToLevel(3);
    			break;
    		case 270:
    			motaVator.goToLevel(4);
    			break;
    	}
    	switch (xboxMinor.getPOV(0)){
			case 0:
				motaVator.goToLevel(1);
				break;
			case 90:
				motaVator.goToLevel(2);
				break;
			case 180:
				motaVator.goToLevel(3);
				break;
			case 270:
				motaVator.goToLevel(4);
				break;
    	}
    	
    	if(xboxMinor.getRawAxis(2) > .5 && !elevatorHeightChangedOneShot){
    		motaVator.goToHeight(-16);
    		elevatorHeightChangedOneShot = true;
    	}

    	if(xboxMinor.getRawAxis(3) > .5 && !elevatorHeightChangedOneShot){
    		motaVator.goToHeight(-40);
    		elevatorHeightChangedOneShot = true;
    	}
    	
    	if(xboxMinor.getButtonPressedOneShot(6)){
    		motaVator.goToHeight(-10.5);
    	}
	    
    	if (xboxMinor.getRawAxis(2) < .5 && xboxMinor.getRawAxis(3) < .5)  {
    		elevatorHeightChangedOneShot = false;
    	}
    	
    	
	    robotDrive.correctedMecanumDrive(driveValue[0], driveValue[1], driveValue[2], 0.0, Constants.ftbCorrectionNoTote);
	    

    	//logger.logEntry();
	    //BackL.logEntry();

    }
    
    
    
    public void testInit(){
    	navx.zeroYaw();
    	testTick = 0;

    }
        
    private int testTick;
  
    public void testPeriodic() {
    	
    	//Manual Elevator Control for re-widing cable - Button 1 & 4
	    if((xboxSupremeController.getRawButton(1) || xboxMinor.getRawButton(1))){
    		motaVator.maintenanceMode(.25);
    		maintenanceModeWasRun = true;
    	} else if((xboxSupremeController.getRawButton(4) || xboxMinor.getRawButton(4))){
    		motaVator.maintenanceMode(-.25);
    		maintenanceModeWasRun = true;
    	}  else {
    		motaVator.maintenanceMode(0.0);
    	}

    	if (++testTick >=50) {
    		testTick = 0;
    		System.out.println("Top Limit Switch: " + motaVator.topSwitch.get());
    		System.out.println("Side Ultra Range: " + ultraSide.getRangeInches());
    		System.out.println("Front Ultra Range: " + ultraFront.getRangeInches());
    		
    	}
    	
    	
/*
    	if(xboxSupremeController.getToggleButton(5) || xboxMinor.getToggleButton(5)){
        	
	    	if((xboxSupremeController.getButtonPressedOneShot(1) || xboxMinor.getButtonPressedOneShot(1))){
	    		motaVator.goToHeight(-12.0); // lowers elevator
	    		
	    	} else if((xboxSupremeController.getButtonPressedOneShot(4) || xboxMinor.getButtonPressedOneShot(4))){
	    		motaVator.goToHeight(-18.5);  // raises elevator
	    	} 
	    		    
    	} else {
	    
	    if((xboxSupremeController.getRawButton(1) || xboxMinor.getRawButton(1))){
    		
    		motaVator.lowerManual(); // lowers elevator
    	} else if((xboxSupremeController.getRawButton(4) || xboxMinor.getRawButton(4))){
    		motaVator.raiseManual();  // raises elevator
    	} else {
    		motaVator.equilibrium();
    	}
    	}
    	
    	if(xboxSupremeController.getButtonPressedOneShot(2) || (xboxMinor.getButtonPressedOneShot(2))){
    		motaVator.grab(); // open or close arms
    	}
    	
    	if (++testTick >= 20) {
    		System.out.printf("SP: %.8f PV: %.8f, Err: %.8f MV: %.8f\n", motaVator.pid.getSetpoint(), 
    															motaVator.enc.getDistance(), 
    															motaVator.pid.getError(), 
    															motaVator.pid.get());
    		testTick = 0;
    	}
    }
    
    
    public void testPeriodic() {
       /*	
    	double angler = 0.0; //xboxSupremeController.getToggleButton(8) ? navx.getYaw() : 0.0;
    	
    	if (xboxSupremeController.getOneShotButton(7)){
    		navx.zeroYaw();
    	}
    	
    	gearShiftSolenoid.set(false);
    	FrontL.setGearPID(false);
    	FrontR.setGearPID(false);
    	BackL.setGearPID(false);
    	BackR.setGearPID(false);
    	FrontL.enableLogging(xboxSupremeController.getToggleButton(8));
    	FrontR.enableLogging(xboxSupremeController.getToggleButton(8));
    	BackL.enableLogging(xboxSupremeController.getToggleButton(8));
    	BackR.enableLogging(xboxSupremeController.getToggleButton(8));

    	if (xboxSupremeController.getRawButton(4)) {
            y = .4;
            x = 0.0;
            z = angler * -.008;
        } else if (xboxSupremeController.getRawButton(1)) {
            y = -.4;
            x = 0.0;
            z = angler * -.008;
        } else if (xboxSupremeController.getRawButton(3)) {
            y = 0.0;
            x = .4;
            z = angler * -.008;
        } else if (xboxSupremeController.getRawButton(2)) {
            y = 0.0;
            x = -.4;
            z = angler * -.008;
        } else {
            y = 0.0;
            x = 0.0;
            z = 0.0;
        }

    	robotDrive.mecanumDrive_Cartesian(x, y, z, 0);
        //robotDrive.correctedMecanumDrive(x, y, z, 0.0, -.15);
    	FrontL.logEntry();
    	FrontR.logEntry();
    	BackL.logEntry();
    	BackR.logEntry(); */
    }
	    
    
   
    
    
}
