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
	
	void patternWaveRoll(){
		setMode(2);
	}
	
	void patternWaveInvert(){
		setMode(3);
	}
	
	void patternCenterFillGreen(){
		setMode(4);
	}
	
	void patternCenterFillGold(){
		setMode(5);
	}
	
	void patternCenterPulseGreen(){
		setMode(6);
	}
	
	void patternFlashGreenGold(){
		setMode(7);
	}
	
	void patternFlashRed(){
		setMode(8);
	}
}
