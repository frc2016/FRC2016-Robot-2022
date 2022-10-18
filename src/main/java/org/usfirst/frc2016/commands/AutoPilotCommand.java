package org.usfirst.frc2016.commands;

import org.usfirst.frc2016.Robot;
import edu.wpi.first.wpilibj.command.CommandGroup;

// class to start AutoPilot sequences
public class AutoPilotCommand extends CommandGroup {
	String sequenceKey;
	Boolean checkFileModified = false;

	public AutoPilotCommand(String sequenceKey) {
		this.sequenceKey = sequenceKey;

		setRequires();
	}

	public AutoPilotCommand(String sequenceKey, Boolean checkFileModified) {
		this.sequenceKey = sequenceKey;
		this.checkFileModified = checkFileModified;

		setRequires();
	}

	protected void setRequires() {

		requires(Robot.driveTrainSRX);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (!sequenceKey.isEmpty()) {
			Robot.autoPilotRobot.initialize(sequenceKey, checkFileModified);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		Robot.autoPilotRobot.execute();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.autoPilotRobot.isFinished();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.autoPilotRobot.end();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.autoPilotRobot.interrupted();
	}
}
