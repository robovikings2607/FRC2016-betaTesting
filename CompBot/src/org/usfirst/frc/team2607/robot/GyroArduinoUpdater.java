package org.usfirst.frc.team2607.robot;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class GyroArduinoUpdater extends Thread {
	IMUAdvanced navx;
	boolean good;
	double startAngle = 0;
	
	int address = 4;
	I2C i2c;
	
	int flashCoutner = 0;
	boolean flashPosition;
	
	GyroArduinoUpdater(IMUAdvanced navx, boolean good){
		this.navx = navx;
		this.good = good;
		i2c = new I2C(Port.kOnboard, address);
		
		this.startAngle = this.navx.getYaw();
		
		this.start();
	}
	
	GyroArduinoUpdater(IMUAdvanced navx, boolean good, int address){
		this(navx, good);
		
		this.address = address;		
	}
	
	public void run() {
		while (true){
			//System.out.println("Yeah");
			if(good){
				if (startAngle - navx.getYaw() - .13 > 0 ) setLeft();
				else if (startAngle - navx.getYaw() + .13 < 0 ) setRight();
				else setCenter();
				//System.out.println("Inside!");
			} else {
				setFlash();
				//System.out.println("NO!");
			}
			
			try {
				Thread.sleep(80);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	void setLEDs(boolean[] lights){
		byte i = 0;
		
		if (lights[0]) i |= 1 << 0;
		if (lights[1]) i |= 1 << 1;
		if (lights[2]) i |= 1 << 2;
		
		byte[] toSend = {i};
		
		i2c.transaction(toSend, toSend.length, null, 0);
	}
	
	void setLeft(){
		setLEDs(new boolean[] {true, true, false});
	}
	
	void setRight(){
		setLEDs(new boolean[] {false, true, true});
	}
	
	void setCenter(){
		setLEDs(new boolean[] {true, true, true});
	}
	
	void setFlash(){
		if (flashCoutner++ > 5){
			flashCoutner = 0;
			flashPosition = !flashPosition;
		}
		
		if(flashPosition) setLEDs(new boolean[] {true, true, true});
		else setLEDs(new boolean[] {false, false, false});
	}

}
