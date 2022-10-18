package org.usfirst.frc2016.commands;

import org.usfirst.frc2016.Robot;
import org.usfirst.frc2016.AutoPilot.AutoPilotMethod;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;
import java.util.Timer;
import java.util.TimerTask;

// class to start AutoPilot sequences
public class PathPlannerCommand extends CommandGroup {
	String path;
	boolean reloadPaths = false;
	boolean useMP = false;

	public PathPlannerCommand(String path, boolean reloadPaths, boolean useMP) {
		this.path = path;
		this.reloadPaths = reloadPaths;
		this.useMP = useMP;

		requires(Robot.driveTrainSRX);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (reloadPaths) {
			Robot.pathPlanner.initializeRobotPaths();
		}
		Robot.pathPlanner.start(path, useMP);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		Robot.pathPlanner.execute();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return Robot.pathPlanner.isFinished();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.pathPlanner.end();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.pathPlanner.interrupted();
	}
}
