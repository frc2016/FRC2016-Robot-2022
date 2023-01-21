package org.usfirst.frc2016;

// import java.util.Objects;
import java.util.HashMap;
import java.util.Map;
import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc2016.AutoPilot.*;
import org.usfirst.frc2016.commands.*;

import edu.wpi.first.wpilibj.command.Command;

// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPilotRobot {
	private AutoPilot autoPilot;
	// private String lastStatus;
	private Timer driveUpdateTimer;
	private int nDriveUpdateMs = 10;
	private String moveFilePath = "/c/";
	private Map<String, Command> mapCommands;

	// sequence base names defined in AutoPilot.txt
	public static String MoveOffLine = "Move Off Line";
	public static String UnderTrench = "Under Trench";
	public static String toMiddle = "middle balls";

	public AutoPilotRobot() {
		autoPilot = new AutoPilot();
		autoPilot.addMethods(this);

		mapCommands = new HashMap<>();

		addCommand(new IntakeArmUp());
		addCommand(new IntakeArmDown());
		addCommand(new IntakeArmStop());
		addCommand(new BallPickup(1.5));
	}

	private void addCommand(String name, final Command cmd) {
		mapCommands.put(name, cmd);
	}

	private void addCommand(final Command cmd) {
		String name = cmd.getClass().getSimpleName();
		addCommand(name, cmd);
	}

	public void loadFile() {
		autoPilot.setTracing(true);
		listCommands();
		autoPilot.loadFile(moveFilePath + "AutoPilot.txt");
		SmartDashboard.putString("AutoPilotStatus", autoPilot.getStatus());
	}

	public String[] sequenceNames() {
		return autoPilot.sequenceNames();
	}

	public enum MagState {
		Fwd, Rev, Stop
	}

	public enum BallIntakePivotState {
		Up, Down, Stop
	}

	public enum BallIntakeSpinState {
		In, Out, Stop
	}

	public enum ShooterState {
		Low, High, Distance, In, Stop
	}

	public void initialize(String selectName, boolean checkFileModified) {
		// get the game data
		// String rawGameData = DriverStation.getInstance().getGameSpecificMessage();

		// // replace sequence of 'stars' with game data
		String sequenceName = selectName;
		// sequenceName = sequenceName.replace("**", rawGameData.substring(0,2));
		// sequenceName = sequenceName.replace("*", rawGameData.substring(0,1));

		Robot.driveTrainSRX.resetEncoders();

		Double[] distances = new Double[AutoPilot.MOTOR_MAX];
		distances[AutoPilot.MOTOR_LEFT] = 0.0;
		distances[AutoPilot.MOTOR_RIGHT] = 0.0;

		// Double[] distances = getDriveDistances();
		// Robot.driveTrainSRX.goToDistance(
		// distances[AutoPilot.MOTOR_LEFT],
		// distances[AutoPilot.MOTOR_RIGHT]);

		// autoPilot.setTracing(false);
		autoPilot.startSequence(sequenceName, distances, checkFileModified);
		startTimerTask();
	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {

		Robot.driveTrainSRX.pingDifferentialDrive();
		SmartDashboard.putString("AutoPilotStatus", autoPilot.getStatus());
		autoPilot.logPerformance();

		// if (!autoPilot.isTracing()) {
		// String status = autoPilot.getStatus();
		// if (!Objects.equals(lastStatus, status)) {
		// lastStatus = status;
		// System.out.println(lastStatus);
		// }
		// }

	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return autoPilot.isFinished();
	}

	public boolean isRunning() {
		return !autoPilot.isFinished();
	}

	// Called once after isFinished returns true
	public void end() {
		autoPilot.end();
		shutdown();
		autoPilot.logSave();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	// --@Override
	public void interrupted() {
		autoPilot.interrupted();
		shutdown();
	}

	// called via reflection to get the current distance from the drive train
	@AutoPilotMethod(argHint = "")
	public Double[] getDriveDistances() {
		Double[] distances = new Double[AutoPilot.MOTOR_MAX];
		distances[AutoPilot.MOTOR_LEFT] = 0.0;
		distances[AutoPilot.MOTOR_RIGHT] = 0.0;

		distances[AutoPilot.MOTOR_LEFT] = Robot.driveTrainSRX.getLeftDistance();
		distances[AutoPilot.MOTOR_RIGHT] = Robot.driveTrainSRX.getRightDistance();

		return distances;
	}

	public void shutdown() {
		killTimerTask();
		// stop all the robot parts
		Robot.driveTrainSRX.driveStop();
		StartCommand("IntakeArmStop");

		BallIntakeSpin(BallIntakeSpinState.Stop);
		BallIntakePivot(BallIntakePivotState.Stop);
	}

	private void startTimerTask() {
		killTimerTask();
		driveUpdateTimer = new Timer();
		driveUpdateTimer.scheduleAtFixedRate(new DriveTimerTask(), nDriveUpdateMs, nDriveUpdateMs);
	}

	private void killTimerTask() {
		if (null != driveUpdateTimer) {
			driveUpdateTimer.cancel();
			driveUpdateTimer.purge();
			driveUpdateTimer = null;
		}
	}

	class DriveTimerTask extends TimerTask {
		@Override
		public void run() {
			if (autoPilot.isFinished()) {
				killTimerTask();
			} else {
				double[] driveDistances = autoPilot.execute();
				if (null != driveDistances) {

					Robot.driveTrainSRX.goToDistance(driveDistances[AutoPilot.MOTOR_LEFT],
							driveDistances[AutoPilot.MOTOR_RIGHT]);

					// autoPilot.traceMessage("Cmd %.1f %.1f Pos %.1f %.1f ",
					// driveDistances[AutoPilot.MOTOR_LEFT],
					// driveDistances[AutoPilot.MOTOR_RIGHT], Robot.driveTrainSRX.getLeftDistance(),
					// Robot.driveTrainSRX.getRightDistance());
				}
			}
		}
	}

	@AutoPilotMethod(argHint = "( name ) : starts named command")
	public void StartCommand(String name) {

		Command cmd = mapCommands.getOrDefault(name, null);
		if (cmd != null) {
			// start the named command
			cmd.start();
		} else {
			System.out.println(String.format("Command named '%s' not found. ", name));
			listCommands();
		}
	}

	private void listCommands() {
		String prefix = "// ";
		System.out.println(prefix + " Vaild commands for 'StartCommand' are:");

		String[] arr = new String[mapCommands.size()];
		mapCommands.keySet().toArray(arr);
		Arrays.sort(arr);

		StringBuilder line = new StringBuilder();
		line.append(prefix);

		for (String key : arr) {
			line.append(" ");
			line.append(key);
			if (line.length() > 80) {
				System.out.println(line.toString());

				line = new StringBuilder();
				line.append(prefix);
			}
		}

		if (line.length() > prefix.length()) {
			System.out.println(line.toString());
		}
	}

	@AutoPilotMethod(argHint = "( Up|Down|Stop ) : pivots intake")
	public void BallIntakePivot(BallIntakePivotState newIntakePivot) {
		switch (newIntakePivot) {

			case Up:
				Robot.ballIntake.armUp();
				// StartCommand("IntakeArmUp");
				break;

			case Down:
				Robot.ballIntake.armDown();
				// StartCommand("IntakeArmDown");
				break;

			default:
			case Stop:
				Robot.ballIntake.armStop();
				// StartCommand("IntakeArmStop");
				break;

		}
	}

	@AutoPilotMethod(argHint = "( In|Out|Stop ) : spins intake")
	public void BallIntakeSpin(BallIntakeSpinState newIntakeSpin) {
		switch (newIntakeSpin) {

			case In:
				Robot.ballIntake.brushIn();
				break;

			case Out:
				Robot.ballIntake.brushOut();
				break;

			default:
			case Stop:
				Robot.ballIntake.brushStop();
				break;

		}
	}
}
