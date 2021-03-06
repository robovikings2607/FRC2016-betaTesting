package org.usfirst.frc.team2607.robot;

import edu.wpi.first.wpilibj.Joystick;

public class robovikingStick extends Joystick{


private int previousState;
private boolean[] buttonStates;

public robovikingStick(int port) {
    super(port);
    previousState = 0;
    buttonStates = new boolean[16];
    for (int i = 0; i < buttonStates.length; i++) buttonStates[i] = false;
}

public boolean getOneShotButton(int buttonNumber) {
    int bitValue = 0x1 << (buttonNumber - 1);
    boolean retValue = false;
    
    boolean buttonWasOff = (bitValue & previousState) == 0;
    boolean buttonIsOn = getRawButton(buttonNumber);
    
    if (buttonWasOff && buttonIsOn) retValue = true;
    if (buttonIsOn) previousState = previousState | bitValue;
    if (!buttonIsOn) previousState = previousState & ~bitValue;
    
    return retValue;
}

public boolean getToggleButton(int buttonNumber) {
	int btn = buttonNumber - 1;
	if (getOneShotButton(buttonNumber)) buttonStates[btn] = !buttonStates[btn];
	return buttonStates[btn];
}
}