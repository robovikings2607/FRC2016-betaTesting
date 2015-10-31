package org.usfirst.frc.team2607.robot;

import java.io.File;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.DriverStation;

public class Logger extends Thread {
	private PrintWriter logFile = null;
	private boolean loggingEnabled = false;
	String deviceName;
	
	Robot theBot;
	
	
	@Override
	public void run(){
		while (true){
			logEntry();
			try {Thread.sleep(10); } catch (Exception e) {}
		}
		
	}
	
	public Logger(Robot robot){
		theBot = robot;					
	}
	
	 public void enableLogging(boolean enable) {
	    	if (enable && !loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			logFile = null;
	    		}
	    		try {
	    			String s = "/home/lvuser/" + "MatchFiles" + "." + System.currentTimeMillis() + ".csv";
	    			logFile = new PrintWriter(new File(s));
	    			logFile.println("Time,Auto,Tele,navX Cal,navX Conn,navX Yaw");
	    		} catch (Exception e) {}
	    	} 
	    	
	    	if (!enable && loggingEnabled) {
	    		if (logFile != null) {
	    			logFile.close();
	    			logFile = null;
	    		}
	    	}
	    	
	    	loggingEnabled = enable;
	    }
	 
	 public void logEntry() {
	        if (loggingEnabled) {
	        	logFile.println(System.currentTimeMillis() + "," +
	        					theBot.inAuto + "," + 
	        					theBot.inTeleop + "," +
	        				    theBot.navx.isCalibrating() + "," + 
	        					theBot.navx.isConnected() + "," + 
	        				    theBot.navx.getYaw());
	        	logFile.flush();
	        }
	    }

}
