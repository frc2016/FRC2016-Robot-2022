package org.usfirst.frc2016;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class MMW_DifferentialDrive extends DifferentialDrive {

	public MMW_DifferentialDrive(MotorController leftMotor, MotorController rightMotor) {
		super(leftMotor, rightMotor);
	}

	public void pingMotorSafety() {
		feed();
	}

}
