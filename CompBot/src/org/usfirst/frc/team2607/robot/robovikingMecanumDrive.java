package org.usfirst.frc.team2607.robot;
import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class robovikingMecanumDrive extends RobotDrive {
    static final int kFrontLeft_val = 0;
    static final int kFrontRight_val = 1;
    static final int kRearLeft_val = 2;
    static final int kRearRight_val = 3;
    private static int gyroResetTick = 0;
    private IMUAdvanced navx;
    private boolean needGyroReset;
    
    private SpeedController FrontL, RearR, FrontR, RearL;
    
	public robovikingMecanumDrive(SpeedController frontLeftMotor,
			SpeedController rearLeftMotor, SpeedController frontRightMotor,
			SpeedController rearRightMotor,
			IMUAdvanced n) {
		super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		navx = n;
		needGyroReset = true;
		// TODO Auto-generated constructor stub
	}

	public void correctedMecanumDrive(double x, double y, double rotation, double gyroAngle, double frontBackRatio){
		if(!kMecanumCartesian_Reported) {
            UsageReporting.report(tResourceType.kResourceType_RobotDrive, getNumMotors(), tInstances.kRobotDrive_MecanumCartesian);
            kMecanumCartesian_Reported = true;
        }
        double xIn = x;
        double yIn = y;
        // Negate y for the joystick.
        yIn = -yIn;
        // Compensate for gyro angle.
        double rotated[] = rotateVector(xIn, yIn, gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];
        
        if (DriverStation.getInstance().isAutonomous()) gyroResetTick = 999;
        if ((xIn != 0.0 || yIn != 0.0) && rotation == 0.0 && DriverStation.getInstance().isAutonomous()) {
        	if (needGyroReset && ++gyroResetTick >= 3) {
        		navx.zeroYaw();
        		needGyroReset = false;
        		gyroResetTick = 0;
        	}
        	if (!needGyroReset) rotation = navx.getYaw() * -.022; //.016 
        } else {
        	needGyroReset = true;
        }
        
        
        double wheelSpeeds[] = new double[kMaxNumberOfMotors];
        wheelSpeeds[kFrontLeft_val] = xIn + yIn + rotation;
        wheelSpeeds[kFrontRight_val] = -xIn + yIn - rotation;
        wheelSpeeds[kRearLeft_val] = -xIn + yIn + rotation;
        wheelSpeeds[kRearRight_val] = xIn + yIn - rotation;

        normalize(wheelSpeeds);

        byte syncGroup = (byte)0x80;
        
        if(frontBackRatio > 0){
	        m_frontLeftMotor.set((1-frontBackRatio) * wheelSpeeds[kFrontLeft_val] * m_maxOutput, syncGroup);
	        m_frontRightMotor.set((1-frontBackRatio)* wheelSpeeds[kFrontRight_val] * m_maxOutput, syncGroup);
	        m_rearLeftMotor.set(wheelSpeeds[kRearLeft_val] *  m_maxOutput, syncGroup);
	        m_rearRightMotor.set(wheelSpeeds[kRearRight_val] * m_maxOutput, syncGroup);
        } else {
        	m_frontLeftMotor.set(wheelSpeeds[kFrontLeft_val] *  m_maxOutput, syncGroup);
            m_frontRightMotor.set(wheelSpeeds[kFrontRight_val] * m_maxOutput, syncGroup);
            m_rearLeftMotor.set((1-Math.abs(frontBackRatio))* wheelSpeeds[kRearLeft_val] * m_maxOutput, syncGroup);
            m_rearRightMotor.set((1-Math.abs(frontBackRatio)) * wheelSpeeds[kRearRight_val] * m_maxOutput, syncGroup);

        }
        
        if (m_safetyHelper != null) m_safetyHelper.feed();
	}
	
	public void correctedMecanumDrivePolar(double magnitude, double direction, double rotation, double frontBackRatio) {
        if(!kMecanumPolar_Reported) {
            UsageReporting.report(tResourceType.kResourceType_RobotDrive, getNumMotors(), tInstances.kRobotDrive_MecanumPolar);
            kMecanumPolar_Reported = true;
        }
        double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;
        // Normalized for full power along the Cartesian axes.
        magnitude = limit(magnitude) * Math.sqrt(2.0);
        // The rollers are at 45 degree angles.
        double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double wheelSpeeds[] = new double[kMaxNumberOfMotors];
        wheelSpeeds[kFrontLeft_val] = (sinD * magnitude + rotation);
        wheelSpeeds[kFrontRight_val] = (cosD * magnitude - rotation);
        wheelSpeeds[kRearLeft_val] = (cosD * magnitude + rotation);
        wheelSpeeds[kRearRight_val] = (sinD * magnitude - rotation);

        normalize(wheelSpeeds);

        byte syncGroup = (byte)0x80;
        
        if(frontBackRatio > 0){
	        m_frontLeftMotor.set((1-frontBackRatio) * wheelSpeeds[kFrontLeft_val] * m_maxOutput, syncGroup);
	        m_frontRightMotor.set((1-frontBackRatio)* wheelSpeeds[kFrontRight_val] * m_maxOutput, syncGroup);
	        m_rearLeftMotor.set(wheelSpeeds[kRearLeft_val] * m_maxOutput, syncGroup);
	        m_rearRightMotor.set(wheelSpeeds[kRearRight_val] * m_maxOutput, syncGroup);
        } else {
        	m_frontLeftMotor.set(wheelSpeeds[kFrontLeft_val] * m_maxOutput, syncGroup);
            m_frontRightMotor.set(wheelSpeeds[kFrontRight_val] * m_maxOutput, syncGroup);
            m_rearLeftMotor.set((1-Math.abs(frontBackRatio))* wheelSpeeds[kRearLeft_val] * m_maxOutput, syncGroup);
            m_rearRightMotor.set((1-Math.abs(frontBackRatio)) * wheelSpeeds[kRearRight_val] * m_maxOutput, syncGroup);

        }

        if (m_safetyHelper != null) m_safetyHelper.feed();
    }
	
	public void resetDistance(){
		if (m_frontLeftMotor instanceof WheelRPMController){
			((WheelRPMController) m_frontLeftMotor).resetDistance();
			((WheelRPMController) m_frontRightMotor).resetDistance();
			((WheelRPMController) m_rearLeftMotor).resetDistance();
			((WheelRPMController) m_rearRightMotor).resetDistance();
		}
	}
	
	public int getCount(int i){
		if (m_frontLeftMotor instanceof WheelRPMController){
			switch (i){
			case 0:
				return (int) (((WheelRPMController) m_frontLeftMotor).getCount());
			case 1:
				return (int) (((WheelRPMController) m_frontRightMotor).getCount());
			case 2:
				return (int) (((WheelRPMController) m_rearLeftMotor).getCount());
			case 3: 
				return (int) (((WheelRPMController) m_rearRightMotor).getCount());
			default:
				return 0;
			}
		}
		return 0;
	}
	
	public double getWheelDistance(int i){
		SmartDashboard.putNumber("Lower Distance Test 1", ((WheelRPMController) m_frontLeftMotor).getDistance()  * Constants.driveDistancePerPulse);
		if (m_frontLeftMotor instanceof WheelRPMController){
			switch (i){
			case 0:
				return ((WheelRPMController) m_frontLeftMotor).getDistance() * Constants.driveDistancePerPulse;
			case 1:
				return ((WheelRPMController) m_frontRightMotor).getDistance() * Constants.driveDistancePerPulse;
			case 2:
				return ((WheelRPMController) m_rearLeftMotor).getDistance() * Constants.driveDistancePerPulse;
			case 3: 
				return ((WheelRPMController) m_rearRightMotor).getDistance() * Constants.driveDistancePerPulse;
			default:
				return 0.0;
			}
		}
		return 0.0;
	}
	
	public IMUAdvanced getnavX(){
		return navx;
	}
	
	

}
