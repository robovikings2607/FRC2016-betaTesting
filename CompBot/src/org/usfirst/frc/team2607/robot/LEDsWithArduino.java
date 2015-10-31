package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class LEDsWithArduino {
	int address = 0;
	I2C i2c;

	LEDsWithArduino(int I2CAddress){
		address = I2CAddress;
		
		i2c = new I2C(Port.kOnboard, address);
	}
	
	void setMode(int i){
		byte[] toSend = {(byte) i};
		
		i2c.transaction(toSend, toSend.length, null, 0);
	}
	
	void turnOff(){
		setMode(1);
	}
	
	void fillGreen(){
		setMode(2);
	}
	
	void fillGold(){
		setMode(3);
	}
	
	void centerFillGreen(){
		setMode(4);
	}
	
	void centerFillGold(){
		setMode(5);
	}
	
	void waveInversion(){
		setMode(6);
	}
	
	void waveRoll(){
		setMode(7);
	}
	
	void warningFlash(){
		setMode(8);
	}
	
	void centerPulse(){
		setMode(9);
	}
}
