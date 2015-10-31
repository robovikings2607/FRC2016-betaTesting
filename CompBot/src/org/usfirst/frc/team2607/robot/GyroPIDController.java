package org.usfirst.frc.team2607.robot;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class GyroPIDController extends PIDController {

	public GyroPIDController(double Kp, double Ki, double Kd, double Kf, IMUAdvanced x) {
		super(Kp, Ki, Kd, Kf, new GyroSource(x), new DummyOut());
		
		this.setAbsoluteTolerance(1);
		this.setOutputRange(-0.3, 0.3);
	}

}
