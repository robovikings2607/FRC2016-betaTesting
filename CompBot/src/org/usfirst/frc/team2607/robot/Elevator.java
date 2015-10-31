package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements Runnable {
	
	CANTalon elevatorTalon1, elevatorTalon2;
	DigitalInput topSwitch, bottomSwitch;
	Encoder enc;
	Solenoid arms, breaks, shifter;
	robovikingPIDController pid;
	double raiseSpeed = -.75;
	double lowerSpeed = .65;
	boolean armsFlag = false;
	boolean override = false;
	boolean pidDisabled = false;
	double lastHeight = -100000;
	
	int prevDirection = 0; //-1 = up 1 = down
	
	public Elevator(){
		
		enc = new Encoder(Constants.encoderElevatorChannelA,  Constants.encoderElevatorChannelB, true, EncodingType.k1X);
		elevatorTalon1 = new CANTalon(Constants.talonElevator1);
		elevatorTalon2 = new CANTalon(Constants.talonElevator2);
		topSwitch = new DigitalInput(Constants.topSwitchPort);
		bottomSwitch = new DigitalInput(Constants.bottomSwitchPort);
		arms = new Solenoid(1, Constants.armsChannel);
		breaks = new Solenoid(1, Constants.breaksChannel);
		shifter = new Solenoid(1, Constants.winchChannel);
		shifter.set(false);
		elevatorTalon2.changeControlMode(TalonControlMode.Follower);
		elevatorTalon2.set(Constants.talonElevator1);
		elevatorTalon1.enableBrakeMode(true);
		elevatorTalon2.enableBrakeMode(true);
		enc.setPIDSourceType(PIDSourceType.kDisplacement);
    	enc.setDistancePerPulse(Constants.elevatorDistancePerPulse);
    	enc.reset();
//    	pid = new robovikingPIDController(0.09, 0.0011, 0.0006, enc, elevatorTalon1, bottomSwitch, topSwitch);
    	pid = new robovikingPIDController(0.085, 0.00032, 0.0000, enc, elevatorTalon1, bottomSwitch, topSwitch);
//    	pid.setOutputRange(-.6, .45);
    	pid.setOutputRange(-1, .45);
    	pid.setInputRange(-56.2, 0);
    	disablePID();
    	
    	pid.setAbsoluteTolerance(.8);
	}
	
	public void goToCarryingPos() {
		goToHeight(-3.0);
		enablePID();
	}
	
	public void goToHeight(double h) {
		pid.setSetpoint(-Math.abs(h));
		if(h != lastHeight){
			pid.resetAccumulatedError();
			lastHeight = h;
		}
		enablePID();
		SmartDashboard.putNumber("ElevatorHeight", Math.abs(h));
	}
	
	public void disablePID() {
		if (!pidDisabled) {
			pid.disable();
			pidDisabled = true;
		}
	}
	
	public void enablePID() {
		if (pidDisabled) {
			pid.enable();
			pidDisabled = false;
		}
	}
	
	public double getHeight(){
		return enc.getDistance();
	}
	
	public double getToteHeight(){
		return (getHeight()/12.1);
	}
	
	public void resetEncoder(){
		enc.reset();
	}
	
	@Deprecated
	public void raise(){
		pid.setSetpoint(getHeight() - 12.1);
		enablePID();
	}
	
	@Deprecated
	public void lower(){
		pid.setSetpoint(getHeight() + 12.1);
		enablePID();
	}
	
	/**
	 * @param i Level to set elevator to (0 - 4)
	 */
	public void goToLevel(int i){
		switch (i){
		case 0:
			goToHeight(-0);
			break;
		case 1:
			goToHeight(-54.2);
			break;
		case 2:
			//goToHeight(-3.1);
			break;
		case 3:
//			goToHeight(-35.5);
			//goToHeight(lastHeight + .5);
			//41 inches is height for tote loading
			//goToHeight(-25);
			goToHeight(-0);
			break;
		case 4:
			//goToHeight(-54.2);
			break;
		default:
			System.err.println("Seriously?");
		}
		
		enablePID();
	}
	
	public void raiseManual(){
		prevDirection = -1;
		
		disablePID();
		if(topSwitch.get()){
		elevatorTalon1.set(raiseSpeed);
		} else {
		 elevatorTalon1.set(0.0);
		}
	}
	
	public void lowerManual(){
		prevDirection = 1;
		
		disablePID();
		if (bottomSwitch.get()) {
			elevatorTalon1.set(lowerSpeed);
		} else {
			elevatorTalon1.set(0.0);
		}
	}
	
	public void maintenanceMode(double speed) {
		disablePID();
		elevatorTalon1.set(speed);
	}
	
	public void equilibrium(){
		disablePID();
		elevatorTalon1.set(0.0);
	}
	
	public void holdCurrentPosition() {
		double curPos = enc.getDistance();
		
		if (prevDirection == -1){
			prevDirection = 0;
			goToHeight(curPos - 1.2); // -1 since more neg means higher
		} else if (prevDirection == 1){
			prevDirection = 0;
			goToHeight(curPos + .5); // +1 since more pos means lower
		} else {
			goToHeight(curPos);
		}
	}
	
	public void grab(){
		armsFlag = !armsFlag;
		arms.set(armsFlag);
	}
	
	public void toggleOverride(){
		override = !override;
	}
	
	public boolean getOverride(){
		return override;
	}


	@Override
	public void run() {
		while(true) {
			if (!bottomSwitch.get() && (elevatorTalon1.get() > 0)){
				disablePID();
				elevatorTalon1.set(0.0);	
			}
			if (!topSwitch.get() && (elevatorTalon1.get() < 0)){
				disablePID();
				elevatorTalon1.set(0.0);	
			}
			
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {}
		
		}	
	}
		
		
}
	


