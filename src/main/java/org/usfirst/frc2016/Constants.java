package org.usfirst.frc2016;

public class Constants {
	public final static int pidSlot0 = 0;
	public final static int timeoutMs = 0;

	public class DeviceId {

		public final static int pwrDistribution = 0; // PDP

		// motors
		public final static int driveLeft = 1; // falcon 500
		public final static int driveLeftFollower = 2; // falcon 500
		public final static int driveRight = 3; // falcon 500
		public final static int driveRightFollower = 4; // falcon 500

		public final static int tossUpper = 10; // falcon 500
		public final static int tossLower = 11; // falcon 500
		public final static int tossGate = 12; // falcon 500

		public final static int intakeBrush = 20; // sparkmax + neo 550
		public final static int intakeArm = 21; // sparkmax + neo

		public final static int hangLeft = 30; // falcon 500
		public final static int hangRight = 31; // falcon 500
		public final static int hangFlop = 32; // sparkmax + neo 550

	}

	public class DigitalInputs {
		public final static int readyToToss = 0; // ball sensor right before ball goes into toss wheels

	}

	/**
	 * How many sensor units per rotation.
	 * Using Talon FX Integrated Sensor.
	 * 
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	public final static int kSensorUnitsPerRotation = 2048;

	/**
	 * This is a property of the Pigeon IMU, and should not be changed.
	 */
	public final static double kPigeonUnitsPerRotation = 8192.0;

	/**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;

	/**
	 * Base trajectory period to add to each individual
	 * trajectory point's unique duration. This can be set
	 * to any value within [0,255]ms.
	 */
	public final static int kBaseTrajPeriodMs = 0;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop
	 * kP kI kD kF Iz PeakOut
	 */
	public final static Gains kGains_Distanc = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
	public final static Gains kGains_Turning = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
	public final static Gains kGains_Velocit = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50); /*
																										 * measured 6800
																										 * velocity
																										 * units at full
																										 * motor output
																										 */
	public final static Gains kGains_MotProf = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00); /*
																										 * measured 6800
																										 * velocity
																										 * units at full
																										 * motor output
																										 */

	/** ---- Flat constants, you should not need to change these ---- */
	/*
	 * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
	 * can have up to 2 devices assigned remotely to a talon/victor]
	 */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/*
	 * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
	 * is auxiliary
	 */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/*
	 * Firmware currently supports slots [0, 3] and can be used for either PID Set
	 */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;
}