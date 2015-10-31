package org.usfirst.frc.team2607.robot;


public class Constants {

	
	public static int talonFrontLeft = 1;
	public static int talonFrontRight = 2;
	public static int talonBackRight = 3;
	public static int talonBackLeft = 4;
	public static int[] talonCANAddresses = {talonFrontLeft, talonFrontRight, talonBackLeft, talonBackRight};
	
	public static int talonElevator1 = 5;
	public static int talonElevator2 = 6;
	
	public static int gyroChannel = 0;
	public static int bottomSwitchPort = 23;
	public static int topSwitchPort = 22;
	
	public static int ultraFrontPing = 18;
	public static int ultraFrontEcho = 19;
	public static int ultraSidePing = 20;
	public static int ultraSideEcho = 21;
	
	public static int gearShiftChannel = 0;
	public static int winchChannel = 1;
	public static int breaksChannel = 2;
	public static int armsChannel = 3;
	
	public static int encoderFrontRightChannelA = 2;
	public static int encoderFrontRightChannelB = 3;
	public static boolean encoderFrontRightReversed = true;
	
	public static int encoderFrontLeftChannelA = 0;
	public static int encoderFrontLeftChannelB = 1;
	public static boolean encoderFrontLeftReversed = false;
	
	public static int encoderBackRightChannelA = 4; //6;
	public static int encoderBackRightChannelB = 5; //7;
	public static boolean encoderBackRightReversed = false;
	
	public static int encoderBackLeftChannelA = 6; //4;
	public static int encoderBackLeftChannelB = 7; //5;
	public static boolean encoderBackLeftReversed = false;

	public static int encoderElevatorChannelA = 8;
	public static int encoderElevatorChannelB = 9;
	public static boolean encoderElevatorReversed = false;
	
	public static double elevatorDistancePerPulse = 1.043/256;
	public static double driveDistancePerPulse = (1/(256 * 3 * (60.0/24.0))) * (6 * Math.PI); 
	
	public static int[][] encoders = {
		{encoderFrontLeftChannelA, encoderFrontLeftChannelB},
		{encoderFrontRightChannelA, encoderFrontRightChannelB},
		{encoderBackLeftChannelA, encoderBackLeftChannelB},
		{encoderBackRightChannelA, encoderBackRightChannelB}
		};
	
	
	  final static double[][] talonHighGearPIDGains = {
		  
              {.000026, .000015, 0.0},  // leftFrontPID Gains
              {.000026, .000015, 0.0},  // rightFrontPID Gains
              {.000026, .000015, 0.0},  // leftRearPID Gains
              {.000026, .000015, 0.0}   // rightRearPID Gains
};
	  

	  final static double[][] talonLowGearPIDGains = {
              {.000058, .000036, 0.0},  // leftFrontPID Gains
//		  {.000075, .000050, 0.0},
		      {.000058, .000036, 0.0},  // rightFrontPID Gains
              {.000058, .000036, 0.0},  // leftRearPID Gains 
              {.000058, .000036, 0.0}  // rightRearPID Gains                                      
};
	  
	  final static double  talonHighGearMaxSpeed = 18100;
	  final static double talonLowGearMaxSpeed =  8100;
	  
	  final static double ftbCorrectionNoTote = -.13;
	  final static double ftbCorrectionOneTote = -.13; //.15 is really the no tote
	  final static double ftbCorrectionTwoTote = -.09;
	  
	 // final static double kFHighGear = .000028;       // kf = .5 * (1/maxSPeed)
	 // final static double kFLowGear = .000065;    // kf = .5 * (1/maxSpeed)            (low speed)
	  
	  
	  final static double kFHighGear = .000055;       // kf = 1/maxSPeed
	  final static double kFLowGear = .000123;    // kf = 1/maxSpeed (low speed)
	  	  
//		  final static double[][] talonHighGearPIDGains = {
//		  
//              {.000026, .000015, 0.0},  // leftFrontPID Gains
//              {.000026, .000015, 0.0},  // rightFrontPID Gains
//              {.000026, .000015, 0.0},  // leftRearPID Gains
//              {.000026, .000015, 0.0}   // rightRearPID Gains
//};
//
//	  final static double[][] talonLowGearPIDGains = {
//              {.000058, .000036, 0.0},  // leftFrontPID Gains
////		  {.000075, .000050, 0.0},
//		      {.000058, .000036, 0.0},  // rightFrontPID Gains
//              {.000058, .000036, 0.0},  // leftRearPID Gains 
//              {.000058, .000036, 0.0}  // rightRearPID Gains      
	

	  
}

	
