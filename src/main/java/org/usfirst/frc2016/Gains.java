/**
 *  Class that organizes gains used when assigning values to slots
 */
package org.usfirst.frc2016;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;

	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}

	public void setSlot(TalonFX talon, int pidSlot) {
		int timeoutMs = 0;
		talon.config_kP(pidSlot, kP, timeoutMs);
		talon.config_kI(pidSlot, kI, timeoutMs);
		talon.config_kD(pidSlot, kD, timeoutMs);
		talon.config_kF(pidSlot, kF, timeoutMs);
		talon.config_IntegralZone(pidSlot, kIzone, timeoutMs);
		talon.configClosedLoopPeakOutput(pidSlot, kPeakOutput, timeoutMs);
	}

}
