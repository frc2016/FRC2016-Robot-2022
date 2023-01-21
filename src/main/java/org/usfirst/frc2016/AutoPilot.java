package org.usfirst.frc2016;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.File;
import java.io.IOException;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Arrays;
// import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;

//-- import edu.wpi.first.wpilibj.Timer;

public class AutoPilot {

	static final int MOTOR_LEFT = 0;
	static final int MOTOR_RIGHT = 1;
	static final int MOTOR_MAX = 2;

	static final int HIST_SIZE = 50;

	private ImplementedMethod methodGetDistances;
	private final String methodNameGetDistances = "getDriveDistances";

	private final String leftDriveName = "L";
	private final String rightDriveName = "R";
	private final String defaultSequenceName = "****DefaultSequenceName****";
	private final double minMoveValue = .05;

	private long nanoTimeStart; // nanoseconds
	private long nanoTimeLast;
	private double[] lastDistance;
	private int[] updateHistogram = new int[HIST_SIZE];

	private String sourceFilePath;
	private long sourceLastModified;

	private Map<String, NamedSequence> mapSequences;
	private Map<String, ImplementedMethod> mapImplementedMethods;
	private String currentSequenceName = null;

	private ArrayList<SequenceAction> actionList;
	private Integer actionListIndex;
	private SequenceAction currentAction;

	private MotorPosition posLeft;
	private MotorPosition posRight;
	private Double[] moveOffset = new Double[MOTOR_MAX];

	private double cruiseVel = 100;
	private double accel = 200;
	private double tankWidth = 26;
	private double turnRadius = 30;
	private double moveSequenceTime = 0;
	private double actionSequenceTime = 0;

	private String status;
	private Boolean tracing = true;
	private Boolean finished = true;

	private ArrayList<AutoPilotPerformance> robotPerf = new ArrayList<AutoPilotPerformance>();

	public enum MoveEnd {
		GO, STOP
	}

	public enum MoveDirection {
		FORWARD, BACKWARD
	}

	public enum OnOff {
		ON, OFF
	}

	public AutoPilot() {
		mapImplementedMethods = new HashMap<>();
		addMethods(this);
	}

	public void setTracing(boolean trace) {
		this.tracing = trace;
	}

	public boolean isTracing() {
		return tracing;
	}

	public String getStatus() {
		return status;
	}

	public void setStatus(String text) {
		status = text;
		traceMessage(text);
	}

	public void setStatus(String fmt, Object... objects) {
		setStatus(String.format(fmt, objects));
	}

	public double getElapsedSeconds() {
		return (double) (System.nanoTime() - nanoTimeStart) / 1e9;
	}

	public String[] sequenceNames() {
		String[] arr = new String[mapSequences.size()];
		mapSequences.keySet().toArray(arr);
		Arrays.sort(arr);
		return arr;
	}

	// Called repeatedly when the parent command is running
	public double[] execute() {

		try {
			long nanoTimeCurrent = System.nanoTime();
			Double moveSequenceTime = (double) (nanoTimeCurrent - nanoTimeStart) / 1e9;

			lastDistance = updateDistances(moveSequenceTime);

			processCommands(moveSequenceTime);

			Double lastIntervalMs = (double) (nanoTimeCurrent - nanoTimeLast) / 1e6;

			int histIndex = (int) Math.round(lastIntervalMs);
			if (histIndex >= HIST_SIZE)
				histIndex = HIST_SIZE - 1;
			if (histIndex < 0)
				histIndex = 0;
			updateHistogram[histIndex]++;

			nanoTimeLast = nanoTimeCurrent;

			// if (tracing) {
			// traceMessage("%7.3f (+%4.1f) %s %s", moveSequenceTime, lastIntervalMs,
			// posLeft.toString(),
			// posRight.toString());
			// }

		} catch (AutoPilotException e) {
			stop();
			setStatus(e.getMessage());
			lastDistance = null;
		}

		return lastDistance;
	}

	public double[] updateDistances(double moveSequenceTime) {

		double[] distances = new double[MOTOR_MAX];

		distances[MOTOR_LEFT] = posLeft.updatePosition(moveSequenceTime) + moveOffset[MOTOR_LEFT];
		distances[MOTOR_RIGHT] = posRight.updatePosition(moveSequenceTime) + moveOffset[MOTOR_RIGHT];

		return distances;
	}

	public double[] lastVel() {
		double[] vel = new double[] { posLeft.getVel(), posRight.getVel() };
		return vel;
	}

	public double getLeftDistance() {
		return posLeft.getDistance() + moveOffset[MOTOR_LEFT];
	}

	public double getRightDistance() {
		return posRight.getDistance() + moveOffset[MOTOR_RIGHT];
	}

	// Make this return true when parent Command no longer needs to run
	// execute()
	public boolean isFinished() {
		return finished;
	}

	// Called once after isFinished returns true
	public void end() {
		shutdown();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	public void interrupted() {
		shutdown();
	}

	private void shutdown() {
		finished = true;
		if (isTracing()) {
			int sum = 0;
			int total = 0;
			traceMessage("Update interval histogram");
			traceMessage("  UpdateMs,Count");
			for (int i = 0; i < HIST_SIZE; i++) {
				int count = updateHistogram[i];
				if (count > 0) {
					traceMessage("   %d, %d", i, count);
					sum += i * count;
					total += count;
				}
			}
			if (total > 0) {
				traceMessage(" Average: %.1f", (double) sum / (double) total);
			}
		}
	}

	public void traceMessage(String msg) {
		if (tracing) {
			System.out.println(msg);
		}
	}

	public void traceMessage(String fmt, Object... objects) {
		if (tracing) {
			traceMessage(String.format(fmt, objects));
		}
	}

	public void logPerformance() {
		if (finished) {
			return;
		}
		double seconds = getElapsedSeconds();
		robotPerf.add(new AutoPilotPerformance(seconds, lastDistance, lastVel()));
	}

	public void logSave() {
		if (robotPerf.size() > 0) {
			String msg = "";
			try {
				String filename = "/tmp/" + currentSequenceName + "_log.csv";
				FileWriter fw = new FileWriter(filename);

				String header = "time,tpl,tpr,apl,apr,tvl,tvr,avl,avr\n";
				fw.write(header);

				int index = 0;
				for (AutoPilotPerformance p : robotPerf) {

					// write line into log
					fw.write(p.str());

					// ping the drive to keep the watchdog happy
					if (++index % 100 == 0) {
						Robot.driveTrainSRX.pingDifferentialDrive();
					}
				}

				fw.close();
				msg = String.format("%s end @ %3.1f %d pts", filename, getElapsedSeconds(), robotPerf.size());

			} catch (Exception e) {
				msg = "RobotPerf save error: " + e.getMessage();
			}

			robotPerf = new ArrayList<AutoPilotPerformance>();
			traceMessage(msg);
		}
	}

	@AutoPilotMethod(argHint = "( On | Off ) : turn on tracing")
	public void enableTracing(OnOff b) {
		tracing = (b == OnOff.ON);
	}

	long fileLastModified(String filePath) {
		File file = new File(filePath);
		return file.lastModified();
	}

	@SuppressWarnings("resource")
	public void loadFile(String filePath) {

		tracing = false;

		setStatus("AutoPilot loading '%s'", filePath);
		traceMessage("Working Directory1 = " + System.getProperty("user.dir"));

		finished = false;
		nanoTimeStart = System.nanoTime();
		nanoTimeLast = nanoTimeStart;

		cruiseVel = 100;
		accel = 200;
		tankWidth = 26;
		turnRadius = 30;
		Integer lineNumber = 0;
		BufferedReader inputReader = null;
		mapSequences = new HashMap<>();
		String fileLine = "";

		try {
			// Object[] startSequenceParams = new Object[] { new
			// Double[DRIVE_MOTOR_MAX] };
			// thisMethodStartSequence = findPrivateMethod(this,
			// "startSequence", null);
			sourceFilePath = filePath;
			sourceLastModified = fileLastModified(sourceFilePath);

			methodGetDistances = methodFind(methodNameGetDistances);
			if (null == methodGetDistances)
				throw new AutoPilotException("Missing method 'public Double [] %s()'", methodNameGetDistances);

			initializeSequence(defaultSequenceName);

			inputReader = new BufferedReader(new FileReader(filePath));

			// regex pattern to obtain method name and argument list
			Pattern funcPattern = Pattern.compile("([a-zA-Z0-9]+)\\(([ ,.'a-zA-Z0-9\\-\\\"]*)\\)");

			while (!finished && (fileLine = inputReader.readLine()) != null) {
				lineNumber++;

				// strip comments
				int slashes = fileLine.indexOf("//");
				if (slashes >= 0)
					fileLine = fileLine.substring(0, slashes);
				fileLine = fileLine.trim();

				// ignore full comment and blank lines
				if (fileLine.length() == 0)
					continue;

				// split command and string of all args
				Matcher funcMatcher = funcPattern.matcher(fileLine);
				if (!funcMatcher.matches() || funcMatcher.groupCount() != 2) {
					throw new AutoPilotException("missing command name or parens '%s'", fileLine);
				}

				String cmdName = funcMatcher.group(1);
				String[] stringArgs = funcMatcher.group(2).split(",");

				// trim all the arguments
				for (int argIndex = 0; argIndex < stringArgs.length; argIndex++) {
					stringArgs[argIndex] = stringArgs[argIndex].trim();
				}

				String fileLineTrace = String.format("%3d: %s( %s )", lineNumber, cmdName,
						String.join(", ", stringArgs));

				ImplementedMethod method = mapImplementedMethods.getOrDefault(cmdName.toLowerCase(), null);
				if (null == method) {
					throw new AutoPilotException("command '%s' not found", cmdName);
				}

				Object[] objectArgs = method.convertArgs(stringArgs);

				if (this == method.getClassInstance()) {
					// path planning methods are always done at the current moveSequenceTime
					currentAction = new SequenceAction(fileLineTrace, moveSequenceTime);
					traceMessage(">> %.2f %s", currentAction.getScheduledTime(), currentAction.getSourceText());

					// methods in this class are executed for path planning
					method.invoke(objectArgs);

				} else {
					// action methods are done at actionSequenceTime
					currentAction = new SequenceAction(fileLineTrace, actionSequenceTime);
					traceMessage(">> %.2f %s", currentAction.getScheduledTime(), currentAction.getSourceText());

					// methods from other classes are added to the schedule
					currentAction.setMethod(method, objectArgs);

					actionSequenceTime = moveSequenceTime;
				}

				actionListInsert(currentAction);
				currentAction = null;
			}

			newSequence(null);

			inputReader.close();
			double loadTime = (System.nanoTime() - nanoTimeStart) / 1e6;
			setStatus("%d sequences  Load time %.1f ms", mapSequences.size(), loadTime);
			if (tracing) {

				NamedSequence[] arr = new NamedSequence[mapSequences.size()];
				mapSequences.values().toArray(arr);
				Arrays.sort(arr);

				for (NamedSequence seq : arr) {
					traceMessage("   %4.1f: %s", seq.getExecTime(), seq.getName());
				}
			}

			backupSourceFile();

		} catch (Exception e) {
			setTracing(true);
			setStatus("Error %s Line %d: %s", e.getMessage(), lineNumber, fileLine);
			finished = true;
		}

		if (null != inputReader) {
			try {
				inputReader.close();
			} catch (IOException e) {
				traceMessage("Error closeing file " + e.getMessage());
			}
		}

		actionListIndex = 0;
		moveSequenceTime = 0;
		actionSequenceTime = 0;
		nanoTimeStart = System.nanoTime();
		nanoTimeLast = nanoTimeStart;
		finished = true;
	}

	private void actionListInsert(SequenceAction action) {
		// keep the action list sorted in time order
		if (null != action) {

			int insertIndex = actionList.size() - 1;
			for (; insertIndex >= 0; insertIndex--) {
				if (actionList.get(insertIndex).getScheduledTime() <= action.getScheduledTime()) {
					break;
				}
			}
			insertIndex++;
			actionList.add(insertIndex, action);
		}
	}

	private void backupSourceFile() throws IOException {
		String backup = sourceFilePath.replace(".txt", ".bak");
		Files.copy(Paths.get(sourceFilePath), Paths.get(backup), StandardCopyOption.REPLACE_EXISTING);
	}

	private ImplementedMethod methodFind(String name) {
		return mapImplementedMethods.getOrDefault(name.toLowerCase(), null);
	}

	public void addMethods(Object classInstance) {
		// use simple type name
		String typeName = classInstance.getClass().getTypeName();
		String[] names = typeName.split("\\.");
		typeName = names.length > 0 ? names[names.length - 1] : typeName;

		for (Method method : classInstance.getClass().getMethods()) {
			AutoPilotMethod ac = method.getAnnotation(AutoPilotMethod.class);
			if (null != ac) {
				ImplementedMethod im = new ImplementedMethod(typeName, classInstance, method, ac);
				String key = method.getName().toLowerCase();
				mapImplementedMethods.put(key, im);
			}
		}
	}

	@AutoPilotMethod(argHint = "( ) : lists all of the available methods")
	public void listMethods() {
		if (tracing) {
			ImplementedMethod[] arr = new ImplementedMethod[mapImplementedMethods.size()];
			mapImplementedMethods.values().toArray(arr);
			Arrays.sort(arr);

			String lastType = null;
			for (ImplementedMethod im : arr) {
				if (!im.isHidden()) {
					if (lastType != im.getTypeName()) {
						lastType = im.getTypeName();
						traceMessage("// %s Methods:", lastType);
					}
					traceMessage("//   %s", im.toString());
				}
			}
		}
	}

	public void startSequence(Double[] offsetDistances) throws AutoPilotException {
		// set drive starting offsets
		updateHistogram = new int[HIST_SIZE];
		actionListIndex = 0;
		moveSequenceTime = 0;
		actionSequenceTime = 0;

		posLeft.start(offsetDistances[MOTOR_LEFT]);
		posRight.start(offsetDistances[MOTOR_RIGHT]);

		nanoTimeStart = System.nanoTime();
		nanoTimeLast = nanoTimeStart;
		finished = false;
	}

	private void initializeSequence(String sequenceName) throws AutoPilotException {
		if (null == sequenceName)
			sequenceName = defaultSequenceName;

		currentSequenceName = sequenceName;
		moveSequenceTime = 0;
		actionSequenceTime = 0;
		posLeft = new MotorPosition(leftDriveName, moveSequenceTime);
		posRight = new MotorPosition(rightDriveName, moveSequenceTime);
		actionList = new ArrayList<SequenceAction>();

		SequenceAction init = new SequenceAction("--: Start '" + sequenceName + "'", 0.0);
		actionList.add(init);
	}

	@AutoPilotMethod(argHint = "( \"sequenceName\" ) : start a new named sequence")
	public void newSequence(String sequenceName) throws AutoPilotException {

		if (defaultSequenceName != currentSequenceName) {
			SequenceAction end = new SequenceAction("--: end '" + currentSequenceName + "'", moveSequenceTime);
			actionList.add(end);

			mapSequences.put(currentSequenceName,
					new NamedSequence(currentSequenceName, actionList, posLeft, posRight));
		}

		initializeSequence(sequenceName);
		// don't add to action list
		currentAction = null;
	}

	@AutoPilotMethod(argHint = "( \"sequenceName\" ) : adds an existing sequence to the current one")
	public void addSequence(String seqeunceName) throws AutoPilotException {

		NamedSequence namedSequence = mapSequences.get(seqeunceName);
		if (namedSequence == null) {
			new AutoPilotException("Sequence '%' not found", seqeunceName);
		}

		double newMoveSequenceTime = moveSequenceTime;
		for (SequenceAction action : namedSequence.getActionList()) {
			SequenceAction newAction = new SequenceAction(action, moveSequenceTime);
			actionList.add(newAction);
			newMoveSequenceTime = Math.max(newMoveSequenceTime, newAction.getScheduledTime());
		}

		double leftStartDistance = posLeft.getEndDistance();
		for (MoveSegment moveSegment : namedSequence.getPosLeft().moveSegmentList) {
			MoveSegment newMoveSegment = new MoveSegment(moveSegment, moveSequenceTime, leftStartDistance);
			posLeft.addMoveSegment(newMoveSegment);
			newMoveSequenceTime = Math.max(newMoveSequenceTime, newMoveSegment.ptLast.time);
		}

		double rightStartDistance = posRight.getEndDistance();
		for (MoveSegment moveSegment : namedSequence.getPosRight().moveSegmentList) {
			MoveSegment newMoveSegment = new MoveSegment(moveSegment, moveSequenceTime, rightStartDistance);
			posRight.addMoveSegment(newMoveSegment);
			newMoveSequenceTime = Math.max(newMoveSequenceTime, newMoveSegment.ptLast.time);
		}

		moveSequenceTime = newMoveSequenceTime;
		actionSequenceTime = newMoveSequenceTime;
		// don't add to action list
		currentAction = null;
	}

	public boolean startSequence(String sequenceName, Double[] startingDistance, boolean checkFileModified) {
		if (checkFileModified) {
			if (fileLastModified(sourceFilePath) > sourceLastModified) {
				loadFile(sourceFilePath);
			}
		}

		robotPerf = new ArrayList<AutoPilotPerformance>();

		NamedSequence namedSequence = mapSequences.get(sequenceName);
		if (namedSequence == null) {
			setStatus("Sequence '%s' not found", sequenceName);
			finished = true;
			return false;
		}

		if (startingDistance != null) {
			moveOffset[MOTOR_LEFT] = startingDistance[MOTOR_LEFT];
			moveOffset[MOTOR_RIGHT] = startingDistance[MOTOR_RIGHT];
		}

		if (tracing) {

			String msg = String.format("Start position  %.1f %.1f", startingDistance[MOTOR_LEFT],
					startingDistance[MOTOR_RIGHT]);
			traceMessage(msg);
		}

		currentSequenceName = sequenceName;
		moveSequenceTime = 0;
		actionSequenceTime = 0;
		namedSequence.start(new Double[] { 0.0, 0.0 });
		posLeft = namedSequence.getPosLeft();
		posRight = namedSequence.getPosRight();
		actionList = namedSequence.getActionList();
		actionListIndex = 0;
		finished = false;
		nanoTimeStart = System.nanoTime();
		nanoTimeLast = nanoTimeStart;

		return true;
	}

	@AutoPilotMethod(argHint = "( ) : end processing of the file")
	public void stop() {
		finished = true;
	}

	@AutoPilotMethod(argHint = "( deltaSeconds ) : overlap next action with movement, can be negative to overlap with last move")
	public void overlapNextAction(double deltaSeconds) {
		actionSequenceTime += deltaSeconds;
		currentAction.setScheduledTime(actionSequenceTime);
	}

	@AutoPilotMethod(argHint = "( accel ) : sets maximum movement acceleration")
	public void setAccel(double accel) {
		this.accel = accel;
	}

	@AutoPilotMethod(argHint = "( vel ) : sets movement cruise velocity")
	public void setCruiseVel(double vel) {
		this.cruiseVel = vel;
	}

	@AutoPilotMethod(argHint = "( radius ) : sets turn radius")
	public void setTurnRadius(double turnRadius) {
		this.turnRadius = turnRadius;
	}

	@AutoPilotMethod(argHint = "( tankWidth ) : sets the width of the tank drive")
	public void setTankWidth(double tankWidth) {
		this.tankWidth = tankWidth;
	}

	@AutoPilotMethod(argHint = "")
	public void executeZeroPosition() throws AutoPilotException {
		// make the current drive position the 'zero' position of the following
		// moves
		Double[] currentDistances = (Double[]) methodGetDistances.invoke(null);
		if (null == currentDistances || currentDistances.length != MOTOR_MAX) {
			throw new AutoPilotException("%s did not return Double[MOTOR_MAX]", methodGetDistances.getName());
		}
		moveOffset[MOTOR_LEFT] = currentDistances[MOTOR_LEFT] - posLeft.getDistance();
		moveOffset[MOTOR_RIGHT] = currentDistances[MOTOR_RIGHT] - posRight.getDistance();
	}

	@AutoPilotMethod(argHint = "( ) : use the current position as the move starting location")
	public void zeroPosition() throws AutoPilotException {
		// when runnning, call zeroPositionRuntime to make the current drive
		// position effectively 'zero'
		currentAction.setMethod(methodFind("executeZeroPosition"), null);
	}

	@AutoPilotMethod(argHint = "( pauseSeconds ) : pause while stopped")
	public void movePause(double delaySeconds) throws AutoPilotException {
		MovePoint leftLastEndPt = posLeft.getLastEnd(moveSequenceTime);
		MovePoint rightLastEndPt = posRight.getLastEnd(moveSequenceTime);

		if (Math.abs(leftLastEndPt.vel) > minMoveValue || Math.abs(rightLastEndPt.vel) > minMoveValue) {
			throw new AutoPilotException("can only pause when stopped");
		}

		moveEnd(delaySeconds);
	}

	@AutoPilotMethod(argHint = "( deltaPos, Go|Stop ) : straight move, negative deltaPos moves backwards")
	public void move(double deltaPos, MoveEnd endState) throws AutoPilotException {

		double endVel = (endState != MoveEnd.GO) ? 0 : cruiseVel;
		double timeL = posLeft.addTarget(moveSequenceTime, accel, cruiseVel, endVel, deltaPos);
		double timeR = posRight.addTarget(moveSequenceTime, accel, cruiseVel, endVel, deltaPos);

		moveEnd(Math.max(timeL, timeR));
	}

	@AutoPilotMethod(argHint = "( deltaPos, cruise, end ) : straight move with cruise vel, end; cruise set from end.")
	public void moveEx(double deltaPos, double tempCruise, double endVel) throws AutoPilotException {

		double timeL = posLeft.addTarget(moveSequenceTime, accel, tempCruise, endVel, deltaPos);
		double timeR = posRight.addTarget(moveSequenceTime, accel, tempCruise, endVel, deltaPos);

		moveEnd(Math.max(timeL, timeR));

		if (endVel != 0)
			cruiseVel = endVel;
	}

	@AutoPilotMethod(argHint = "( moveLength, jogLength, Go|Stop ) : moves with a side offset")
	public void jogMove(double deltaPos, double jogDelta, MoveEnd endState) throws AutoPilotException {

		if (endState != MoveEnd.STOP) {
			throw new AutoPilotException("only 'Stop' is currently supported");
		}

		double moveLen = Math.abs(deltaPos);
		double moveDir = (deltaPos > 0) ? 1 : -1;

		double jogLen = Math.abs(jogDelta);

		double shortLen = 0.5 * (moveLen - jogLen);
		if (shortLen < 0) {
			throw new AutoPilotException("offset length must be less than total move length");
		}

		double longLen = 0.5 * (moveLen + jogLen);
		double lenRatio = shortLen / longLen;

		double launchTime = cruiseVel / accel;

		double launchLongAccel = accel;
		double launchLongLen = 0.5 * launchLongAccel * launchTime * launchTime;

		double launchShortAccel = launchLongAccel * lenRatio;
		double launchShortLen = 0.5 * launchShortAccel * launchTime * launchTime;
		launchShortLen = launchLongLen * lenRatio;

		double coastLongVel = cruiseVel;
		double coastShortVel = launchShortAccel * launchTime;
		coastShortVel = cruiseVel * lenRatio;

		double swapAccel = accel;
		double swapTime = (coastLongVel - coastShortVel) / swapAccel;
		double swapLen = swapTime * 0.5 * (coastShortVel + coastLongVel);

		double coastTotalLen = moveLen - launchLongLen - swapLen - launchShortLen;
		double coastLongLen = coastTotalLen / (1 + lenRatio);
		double coastShortLen = coastLongLen * lenRatio;

		if (coastLongLen < 0 || coastShortLen < 0) {
			throw new AutoPilotException("move too short or delta too big to perform");
		}

		double coastTime = coastLongLen / coastLongVel;
		coastTime = coastShortLen / coastShortVel;

		double moveTime = 2 * (launchTime + coastTime) + swapTime;

		MotorPosition s2 = jogDelta > 0 ? posLeft : posRight;
		MotorPosition s1 = jogDelta > 0 ? posRight : posLeft;

		MovePoint ptLast1 = s1.getLastEnd(moveSequenceTime);
		MovePoint ptLast2 = s2.getLastEnd(moveSequenceTime);

		double targetPos1 = ptLast1.pos + deltaPos;
		double targetPos2 = ptLast2.pos + deltaPos;

		// accel both sides to target velocities
		ptLast1 = s1.addMoveSegment("launch", ptLast1, moveDir * launchLongAccel, launchTime);
		ptLast2 = s2.addMoveSegment("launch", ptLast2, moveDir * launchShortAccel, launchTime);

		// coast
		ptLast1 = s1.addMoveSegment("cruise", ptLast1, 0, coastTime);
		ptLast2 = s2.addMoveSegment("cruise", ptLast2, 0, coastTime);

		// swap velocities
		ptLast1 = s1.addMoveSegment("swap", ptLast1, -moveDir * swapAccel, swapTime);
		ptLast2 = s2.addMoveSegment("swap", ptLast2, moveDir * swapAccel, swapTime);

		// coast
		ptLast1 = s1.addMoveSegment("cruise", ptLast1, 0, coastTime);
		ptLast2 = s2.addMoveSegment("cruise", ptLast2, 0, coastTime);

		// brake
		ptLast1 = s1.addMoveSegment("brake", ptLast1, -moveDir * launchShortAccel, launchTime);
		ptLast2 = s2.addMoveSegment("brake", ptLast2, -moveDir * launchLongAccel, launchTime);

		s1.checkMoveTarget(ptLast1, targetPos1);
		s2.checkMoveTarget(ptLast2, targetPos2);

		moveEnd(moveTime);
	}

	@AutoPilotMethod(argHint = "( degrees ) : spin in place")
	public void spin(double degrees) throws AutoPilotException {
		double moveLen = 2 * Math.PI * (turnRadius + 0.5 * tankWidth) * degrees / 360;
		double endVel = 0;

		double timeL = posLeft.addTarget(moveSequenceTime, accel, cruiseVel, endVel, moveLen);
		double timeR = posRight.addTarget(moveSequenceTime, accel, cruiseVel, endVel, -moveLen);

		double moveTime = Math.max(timeL, timeR);
		moveEnd(moveTime);
	}

	@AutoPilotMethod(argHint = "( degrees, Forward|Backward, Go|Stop ) : turns along a radius")
	public void turn(double degrees, MoveDirection moveDir, MoveEnd endState) throws AutoPilotException {
		MovePoint leftLastEndPt = posLeft.getLastEnd(moveSequenceTime);
		MovePoint rightLastEndPt = posRight.getLastEnd(moveSequenceTime);
		double moveTime = 0;

		if (Math.abs(leftLastEndPt.vel) < minMoveValue && Math.abs(rightLastEndPt.vel) < minMoveValue) {

			if (MoveEnd.STOP == endState) {
				moveTime = addTurnStopped(degrees, moveDir);
			} else {
				moveTime = addTurnAccel(degrees, moveDir, endState);
			}

		} else if (Math.abs(leftLastEndPt.vel) - cruiseVel < minMoveValue
				&& Math.abs(rightLastEndPt.vel) - cruiseVel < minMoveValue) {

			if (MoveEnd.STOP == endState) {
				moveTime = addTurnAccel(degrees, moveDir, endState);
			} else {
				moveTime = addTurnCoasting(degrees, moveDir);
			}

		} else {
			throw new AutoPilotException("addTurn error: can either turn while at coast vel or stopped");
		}

		moveEnd(moveTime);
	}

	// not moving: spin by move each side in opposite directions
	double addTurnStopped(double degrees, MoveDirection moveDirection) throws AutoPilotException {
		double moveDir = (MoveDirection.FORWARD == moveDirection) ? 1 : -1;

		double longLen = 2 * Math.PI * (turnRadius + 0.5 * tankWidth) * Math.abs(degrees) / 360;
		double shortLen = 2 * Math.PI * (turnRadius - 0.5 * tankWidth) * Math.abs(degrees) / 360;

		MotorPosition longMotor = degrees > 0 ? posLeft : posRight;
		MotorPosition shortMotor = degrees > 0 ? posRight : posLeft;

		double longAccel = accel;
		double shortAccel = longAccel * Math.abs(shortLen / longLen);
		;

		double longCruise = cruiseVel;
		double shortCruise = longCruise * Math.abs(shortLen / longLen);

		double endVel = 0;

		double shortTime = shortMotor.addTarget(moveSequenceTime, shortAccel, shortCruise, endVel, shortLen * moveDir);
		double longTime = longMotor.addTarget(moveSequenceTime, longAccel, longCruise, endVel, longLen * moveDir);

		return Math.max(shortTime, longTime);
	}

	double addTurnCoasting(double degrees, MoveDirection moveDirection) throws AutoPilotException {
		double longLen = 2 * Math.PI * (turnRadius + 0.5 * tankWidth) * Math.abs(degrees) / 360;
		double shortLen = 2 * Math.PI * (turnRadius - 0.5 * tankWidth) * Math.abs(degrees) / 360;
		double moveDir = (MoveDirection.FORWARD == moveDirection) ? 1 : -1;
		double deltaLen = longLen - shortLen;

		// double shortAccelTime = (cruiseVel - minTurnVel) / accel;
		// double shortAccelLen = moveLength(-accel, cruiseVel, shortAccelTime);

		double moveTime = longLen / cruiseVel;
		double shortVel = shortLen / moveTime;

		double shortAccelTime = (cruiseVel - shortVel) / accel;
		double shortAccelLen = 2 * moveLength(-accel, cruiseVel, shortAccelTime);
		double longAccelLen = 2 * cruiseVel * shortAccelTime;

		deltaLen -= longAccelLen - shortAccelLen;
		double coastTime = deltaLen / (cruiseVel - shortVel);

		if (coastTime < minMoveValue) {
			deltaLen = longLen - shortLen;
			shortAccelTime = solveQuadractic(0.5 * -accel, cruiseVel, -deltaLen);
			shortVel = cruiseVel - accel * shortAccelTime;
			shortAccelLen = 2 * moveLength(-accel, cruiseVel, shortAccelTime);
			longAccelLen = 2 * cruiseVel * shortAccelTime;
			deltaLen -= longAccelLen - shortAccelLen;
			coastTime = deltaLen / (cruiseVel - shortVel);
		}
		moveTime = coastTime + 2 * shortAccelTime;

		longLen = cruiseVel * moveTime;
		shortLen = shortVel * coastTime + shortAccelLen;

		MotorPosition shortMotor = degrees > 0 ? posRight : posLeft;
		MotorPosition longMotor = degrees > 0 ? posLeft : posRight;

		MovePoint shortPtLast = shortMotor.getLastEnd(moveSequenceTime);
		MovePoint longPtLast = longMotor.getLastEnd(moveSequenceTime);

		double shortTargetPos = shortPtLast.pos + moveDir * shortLen;
		double longTargetPos = longPtLast.pos + moveDir * longLen;

		shortPtLast = shortMotor.addMoveSegment("brake", shortPtLast, -moveDir * accel, shortAccelTime);
		if (coastTime > 0) {
			shortPtLast = shortMotor.addMoveSegment("cruise", shortPtLast, 0, coastTime);
		}
		shortPtLast = shortMotor.addMoveSegment("accel", shortPtLast, moveDir * accel, shortAccelTime);
		longPtLast = longMotor.addMoveSegment("cruise", longPtLast, 0, moveTime);

		shortMotor.checkMoveTarget(shortPtLast, shortTargetPos);
		longMotor.checkMoveTarget(longPtLast, longTargetPos);

		return moveTime;
	}

	// double[] solveQuadractic(double a, double b, double c) throws
	// AutoPilotException {
	// double[] roots = new double[2];
	// double sq = (b * b) - (4 * a * c);
	// if (sq < 0)
	// throw new AutoPilotException("imagary roots solving quadratic");
	//
	// sq = Math.sqrt(sq);
	// roots[0] = (-b + sq) / (2 * a);
	// roots[1] = (-b - sq) / (2 * a);
	// return roots;
	// }

	double solveQuadractic(double a, double b, double c) throws AutoPilotException {
		double sq = (b * b) - (4 * a * c);
		if (sq < 0)
			throw new AutoPilotException("imagary roots solving quadratic");

		sq = Math.sqrt(sq);
		double r1 = (-b + sq) / (2 * a);
		double r2 = (-b - sq) / (2 * a);
		double r = r1 * r2 < 0 ? Math.max(r1, r2) : Math.min(r1, r2);
		if (r < 0)
			throw new AutoPilotException("negative roots solving quadratic");
		return r;
	}

	/**
	 * Adds a turn that is either accelerating from 'stop to cruise' or from 'cruise
	 * to stop' using the current turn radius, acceleration, and turn arc length
	 * parameters
	 * 
	 * @param degrees       number of degrees turn while moving
	 * @param moveDirection main direction of travel
	 * @param endState      end velocity
	 * @return time needed for the move
	 * @throws AutoPilotException
	 */
	double addTurnAccel(double degrees, MoveDirection moveDirection, MoveEnd endState) throws AutoPilotException {

		double longLen = 2 * Math.PI * (turnRadius + 0.5 * tankWidth) * Math.abs(degrees) / 360;
		double shortLen = 2 * Math.PI * (turnRadius - 0.5 * tankWidth) * Math.abs(degrees) / 360;
		double moveDir = (MoveDirection.FORWARD == moveDirection) ? 1 : -1;

		double launchTime = cruiseVel / accel;
		double launchLen = moveLength(accel, launchTime);

		if (shortLen < launchLen) {
			longLen += launchLen - shortLen;
			shortLen = launchLen;
		}

		double longCruiseLen = longLen - launchLen;
		double longCruiseTime = longCruiseLen / cruiseVel;
		double moveTime = launchTime + longCruiseTime;

		double shortCruiseLen = shortLen - launchLen;
		double shortCruiseVel = shortCruiseLen / longCruiseTime;
		double shortLaunchTime1 = shortCruiseVel / accel;
		double shortLaunchTime2 = (cruiseVel - shortCruiseVel) / accel;

		// double shortLaunchTime = 2 * shortLen / cruiseVel;
		// double shortAccel = cruiseVel / shortLaunchTime;
		// // double shortLaunchLen = moveLength(shortAccel, shortLaunchTime);
		//
		// double shortPauseTime = moveTime - shortLaunchTime;

		MotorPosition shortMotor = degrees > 0 ? posRight : posLeft;
		MotorPosition longMotor = degrees > 0 ? posLeft : posRight;

		MovePoint shortPtLast = shortMotor.getLastEnd(moveSequenceTime);
		MovePoint longPtLast = longMotor.getLastEnd(moveSequenceTime);

		double shortTargetPos = shortPtLast.pos + moveDir * shortLen;
		double longTargetPos = longPtLast.pos + moveDir * longLen;

		if (MoveEnd.GO == endState) {
			// launching from stopped state
			shortPtLast = shortMotor.addMoveSegment("launch", shortPtLast, moveDir * accel, shortLaunchTime1);
			shortPtLast = shortMotor.addMoveSegment("cruise", shortPtLast, 0, longCruiseTime);
			shortPtLast = shortMotor.addMoveSegment("launch", shortPtLast, moveDir * accel, shortLaunchTime2);
			longPtLast = longMotor.addMoveSegment("launch", longPtLast, moveDir * accel, launchTime);
			longPtLast = longMotor.addMoveSegment("cruise", longPtLast, 0, longCruiseTime);
		} else {
			// stopping from cruise velocity
			shortPtLast = shortMotor.addMoveSegment("brake", shortPtLast, -moveDir * accel, shortLaunchTime2);
			shortPtLast = shortMotor.addMoveSegment("cruise", shortPtLast, 0, longCruiseTime);
			shortPtLast = shortMotor.addMoveSegment("brake", shortPtLast, -moveDir * accel, shortLaunchTime1);
			longPtLast = longMotor.addMoveSegment("cruise", longPtLast, 0, longCruiseTime);
			longPtLast = longMotor.addMoveSegment("brake", longPtLast, -moveDir * accel, launchTime);
		}

		shortMotor.checkMoveTarget(shortPtLast, shortTargetPos);
		longMotor.checkMoveTarget(longPtLast, longTargetPos);

		return moveTime;
	}

	private void moveEnd(double moveTime) {
		moveSequenceTime += moveTime;
		actionSequenceTime = moveSequenceTime;
	}

	public static double moveLength(double accel, double time) {
		return 0.5 * accel * time * time;
	}

	public static double moveLength(double accel, double vel, double time) {
		return vel * time + 0.5 * accel * time * time;
	}

	private void processCommands(double timeNow) throws AutoPilotException {

		while (actionListIndex < actionList.size()) {

			currentAction = actionList.get(actionListIndex);
			if (currentAction.scheduledTime > timeNow) {
				currentAction = null;
				break;
			}
			actionListIndex++;

			setStatus("'%s': %5.2f %s", null == currentSequenceName ? "??" : currentSequenceName, timeNow,
					currentAction.getSourceText());

			currentAction.invoke();
		}
		finished = actionListIndex >= actionList.size();
	}

	public class NamedSequence implements Comparable<NamedSequence> {
		private ArrayList<SequenceAction> actionList;
		private String name;
		private MotorPosition posLeft;
		private MotorPosition posRight;

		public NamedSequence(String newName, ArrayList<SequenceAction> newActionList, MotorPosition newPosLeft,
				MotorPosition newPosRight) {
			name = newName;
			actionList = newActionList;
			posLeft = newPosLeft;
			posRight = newPosRight;
		}

		void start(Double[] startPos) {
			posLeft.start(startPos[MOTOR_LEFT]);
			posRight.start(startPos[MOTOR_RIGHT]);
		}

		double getExecTime() {
			double execTime = 0;
			if (null != actionList && actionList.size() > 0) {
				execTime = Math.max(execTime, actionList.get(actionList.size() - 1).scheduledTime);
			}
			if (null != posLeft) {
				execTime = Math.max(execTime, posLeft.getExecTime());
			}
			if (null != posRight) {
				execTime = Math.max(execTime, posRight.getExecTime());
			}
			return execTime;
		}

		String getName() {
			return name;
		}

		ArrayList<SequenceAction> getActionList() {
			return actionList;
		}

		MotorPosition getPosLeft() {
			return posLeft;
		}

		MotorPosition getPosRight() {
			return posRight;
		}

		@Override
		public int compareTo(NamedSequence other) {
			return (int) ((getExecTime() - other.getExecTime()) * 1e5);
		}
	}

	public class ImplementedMethod implements Comparable<ImplementedMethod> {
		private Object classInstance;
		private Method method;
		private AutoPilotMethod annotation;
		private String typeName;
		private String name;

		public ImplementedMethod(String type, Object instance, Method meth, AutoPilotMethod apm) {
			classInstance = instance;
			method = meth;
			annotation = apm;

			name = meth.getName();
			name = name.substring(0, 1).toUpperCase() + name.substring(1);

			typeName = type;
		}

		public String getName() {
			return name;
		}

		public String getTypeName() {
			return typeName;
		}

		public Method getMethod() {
			return method;
		}

		public Boolean isHidden() {
			return 0 == annotation.argHint().length();
		}

		public Object getClassInstance() {
			return classInstance;
		}

		@Override
		public String toString() {
			return String.format("%s %s", name, annotation.argHint());
		}

		public Object invoke(Object[] objectArgs) throws AutoPilotException {
			// invoke the method
			Object returnObject = null;
			try {

				method.setAccessible(true);
				returnObject = method.invoke(classInstance, objectArgs);

			} catch (IllegalAccessException e) {
				throw new AutoPilotException("IllegalAccessException calling %s", toString());

			} catch (IllegalArgumentException e) {
				throw new AutoPilotException("IllegalArgumentException calling %s", toString());

			} catch (InvocationTargetException e) {
				throw new AutoPilotException("%s - %s", name, e.getCause().getMessage());
			}

			return returnObject;
		}

		public Object[] convertArgs(String[] stringArgs) throws AutoPilotException {
			Class<?>[] paramList = method.getParameterTypes();
			Object[] objectArgs = new Object[paramList.length];

			if (stringArgs.length < paramList.length) {
				throw new AutoPilotException("missing arguments");
			}

			// convert parameters to the required types
			for (int argIndex = 0; argIndex < paramList.length; argIndex++) {

				String argText = stringArgs[argIndex];
				Class<?> param = paramList[argIndex];
				String paramName = param.getName();

				if (Objects.equals(paramName, "double")) {

					objectArgs[argIndex] = Double.parseDouble(argText);

				} else if (Objects.equals(paramName, "integer")) {

					objectArgs[argIndex] = Integer.parseInt(argText);

				} else if (Objects.equals(paramName, "java.lang.String")) {

					// strings must be surrounded by quotes
					if (argText.length() < 2 || argText.charAt(0) != '\"'
							|| argText.charAt(argText.length() - 1) != '\"') {
						throw new AutoPilotException("strings must have quotes", param.getName());
					}
					argText = argText.substring(1, argText.length() - 1);
					objectArgs[argIndex] = argText;

				} else if (param.isEnum()) {

					argText = argText.toLowerCase();
					for (Object enumFind : param.getEnumConstants()) {
						String enumText = enumFind.toString().toLowerCase();
						if (Objects.equals(argText, enumText)) {
							objectArgs[argIndex] = enumFind;
							break;
						}
					}
					if (null == objectArgs[argIndex])
						throw new AutoPilotException("enumeration value %s not found in %s", stringArgs[argIndex],
								Arrays.asList(param.getEnumConstants()));

				} else {
					throw new AutoPilotException("parameter type '%s' is not implemented", param.getName());
				}
			}
			return objectArgs;
		}

		@Override
		public int compareTo(ImplementedMethod other) {
			int ret = typeName.compareTo(other.typeName);
			if (0 == ret) {
				ret = name.compareTo(other.name);
			}
			return ret;
		}
	}

	// helper class to contain the parsed command file information
	public class SequenceAction {
		private String sourceText;
		private double scheduledTime;
		private ImplementedMethod implementedMethod;
		private Object[] objectArgs;

		public SequenceAction(String sourceText, double scheduledTime) {
			setSource(sourceText, scheduledTime);
			implementedMethod = null;
			objectArgs = null;
		}

		public SequenceAction(SequenceAction action, double timeOffset) {
			sourceText = action.sourceText;
			scheduledTime = action.scheduledTime + timeOffset;
			implementedMethod = action.implementedMethod;
			objectArgs = action.objectArgs;
		}

		public String getSourceText() {
			return sourceText;
		}

		public double getScheduledTime() {
			return scheduledTime;
		}

		public void setScheduledTime(double scheduledTime) {
			this.scheduledTime = scheduledTime;
		}

		public void setSource(String sourceText, double scheduledTime) {
			this.sourceText = sourceText;
			this.scheduledTime = scheduledTime;
		}

		public void setMethod(ImplementedMethod im, Object[] oArgs) {
			this.implementedMethod = im;
			this.objectArgs = oArgs;
		}

		public void setMethod(ImplementedMethod im, String[] sArgs) throws AutoPilotException {
			this.implementedMethod = im;
			this.objectArgs = im.convertArgs(sArgs);
		}

		public void invoke() throws AutoPilotException {

			if (null != implementedMethod) {
				implementedMethod.invoke(objectArgs);
			}
		}
	}

	// Move profiles for a single motor
	public class MotorPosition {
		// private final double minMoveTime = 0.001;
		private String name;
		private ArrayList<MoveSegment> moveSegmentList;
		private MovePoint ptLast;
		private int moveSegmentIndex;
		private int lastCheckedSize;

		public MotorPosition(String motorName, double startTime) {
			name = motorName;
			ptLast = new MovePoint();
			ptLast.time = startTime;

			moveSegmentList = new ArrayList<MoveSegment>();

			MoveSegment msInit = new MoveSegment("init");
			msInit.accel = 0;
			msInit.ptStart.set(ptLast);
			msInit.ptEnd.set(ptLast);
			moveSegmentList.add(msInit);

			lastCheckedSize = moveSegmentList.size();
			start(0);
		}

		public double getExecTime() {
			return moveSegmentList.get(moveSegmentList.size() - 1).ptEnd.time;
		}

		public double getDistance() {
			return ptLast.pos;
		}

		public double getVel() {
			return ptLast.vel;
		}

		public double updatePosition(double sequenceTime) {
			MoveSegment moveSeq = null;

			// advance segment if needed
			while (moveSegmentIndex < moveSegmentList.size()) {
				moveSeq = moveSegmentList.get(moveSegmentIndex);
				if (moveSeq.ptEnd.time > sequenceTime) {
					break;
				}
				moveSegmentIndex++;
			}

			// calculate location within segment
			if (moveSeq != null) {
				MovePoint pt = moveSeq.getPoint(sequenceTime);
				ptLast.set(pt);
			}

			return getDistance();
		}

		public String toString() {
			double accel = 0;
			String desc = "-";
			if (moveSegmentIndex < moveSegmentList.size()) {
				MoveSegment ms = moveSegmentList.get(moveSegmentIndex);
				accel = ms.accel;
				desc = ms.desc.substring(0, 1);

			}
			return String.format(" %s%s:%2d %7.1f %7.1f %7.1f %7.1f", name, desc, moveSegmentIndex, accel, ptLast.vel,
					ptLast.pos, getDistance());
		}

		public void start(double sequenceOffset) {
			moveSegmentIndex = 0;
			MoveSegment ms = moveSegmentList.get(moveSegmentIndex);
			ptLast.set(ms.ptStart);
		}

		protected MovePoint getLastEnd(double startTime) {
			MovePoint ptLastEnd = moveSegmentList.get(moveSegmentList.size() - 1).ptEnd;
			startTime = Math.max(startTime, ptLastEnd.time);

			if (ptLastEnd.time < startTime) {
				// need a filler segment for the time gap
				ptLastEnd = addMoveSegment("fill", ptLastEnd, 0, startTime - ptLastEnd.time);
			}
			return ptLastEnd;
		}

		public double getEndVelocity() {
			return moveSegmentList.get(moveSegmentList.size() - 1).ptEnd.vel;
		}

		public double getEndDistance() {
			return moveSegmentList.get(moveSegmentList.size() - 1).ptEnd.pos;
		}

		public void checkMoveTarget(MovePoint ptLastEnd, double targetPos) throws AutoPilotException {

			// sanity check for the profile generation:
			// the expected target must match the point generated from the move
			// profiler
			if (Math.abs(targetPos - ptLastEnd.pos) > minMoveValue) {

				// remove entries since the last distance check
				while (moveSegmentList.size() > lastCheckedSize) {
					moveSegmentList.remove(moveSegmentList.size() - 1);
				}

				throw new AutoPilotException("Move path error %s: move end should be %.2f, path at %.2f", name,
						targetPos, ptLastEnd.pos);
			}

			// remove small roundoff errors
			ptLastEnd.pos = targetPos;

			// save size of checked moves
			lastCheckedSize = moveSegmentList.size();
		}

		public double addTargetByVelocity(double startTime, double maxAccel, double maxVel, double deltaPos)
				throws AutoPilotException {

			// ignore a zero length move
			if (Math.abs(deltaPos) < minMoveValue)
				return 0;

			// get the last segment
			MovePoint ptLastEnd = getLastEnd(startTime);

			// calc the move parameters
			double endVel = (deltaPos > 0) ? maxVel : -maxVel;
			double targetPos = ptLastEnd.pos + deltaPos;
			double moveTime = 2 * deltaPos / (ptLastEnd.vel + endVel);
			double moveAccel = (endVel - ptLastEnd.vel) / moveTime;

			if (Math.abs(moveAccel) > maxAccel) {
				throw new AutoPilotException("Move path error: move accel (%.0f) exceeds max accel (%.0f)",
						Math.abs(moveAccel), maxAccel);
			}

			ptLastEnd = addMoveSegment("vel", ptLastEnd, moveAccel, moveTime);

			checkMoveTarget(ptLastEnd, targetPos);
			return moveTime;
		}

		public double addTargetByTime(double startTime, double maxAccel, double moveTime, double deltaPos)
				throws AutoPilotException {

			// ignore a zero length move
			if (Math.abs(deltaPos) < minMoveValue)
				return 0;

			if (moveTime < minMoveValue) {
				throw new AutoPilotException("addTargetByTime: Move path error: move time too small");
			}

			// get the last segment
			MovePoint ptLastEnd = getLastEnd(startTime);

			// calculate the move parameters
			double targetPos = ptLastEnd.pos + deltaPos;
			double targetVel = 2 * deltaPos / moveTime - ptLastEnd.vel;
			double moveAccel = (targetVel - ptLastEnd.vel) / moveTime;

			if (Math.abs(moveAccel) > maxAccel) {
				throw new AutoPilotException("Move path error: move accel (%.0f) exceeds max accel (%.0f)",
						Math.abs(moveAccel), maxAccel);
			}

			// add the segment
			ptLastEnd = addMoveSegment("time", ptLastEnd, moveAccel, moveTime);

			checkMoveTarget(ptLastEnd, targetPos);
			return moveTime;
		}

		public double addTarget(double startTime, double maxAccel, double coastVel, double endVel, double deltaPos)
				throws AutoPilotException {

			// ignore a zero length move
			if (Math.abs(deltaPos) < minMoveValue)
				return 0;

			// get the last segment
			MovePoint ptLastEnd = getLastEnd(startTime);

			// calc the move parameters
			double targetPos = ptLastEnd.pos + deltaPos;
			double moveLen = Math.abs(deltaPos);
			double moveDirection = deltaPos > 0 ? 1 : -1;

			double launchAccel = moveDirection * maxAccel;
			double launchTime = Math.abs((moveDirection * coastVel - ptLastEnd.vel) / maxAccel);
			double launchLen = moveLength(maxAccel, launchTime);
			if (launchLen < minMoveValue) {
				// already moving at coastVel
				launchAccel = 0;
				launchLen = 0;
				launchTime = 0;
			}
			launchLen += Math.abs(ptLastEnd.vel) * launchTime;

			double brakeAccel = -moveDirection * maxAccel;
			double brakeTime = Math.abs((coastVel - endVel) / maxAccel);
			double brakeLen = moveLength(maxAccel, brakeTime);
			if (brakeLen < minMoveValue) {
				// move continuing with the next segment
				brakeAccel = 0;
				brakeLen = 0;
				brakeTime = 0;
			}
			brakeLen += endVel * brakeTime;

			double coastLen = moveLen - launchLen - brakeLen;
			double coastTime = coastLen / coastVel;
			if (coastLen > -minMoveValue) {
				// move needs a coast section
				if (launchLen > minMoveValue) {
					// accelerate (or de-accel) to new velocity
					ptLastEnd = addMoveSegment("launch", ptLastEnd, launchAccel, launchTime);
				}

				// constant velocity coast section
				if (coastLen > minMoveValue) {
					ptLastEnd = addMoveSegment("cruise", ptLastEnd, 0, coastTime);
				}

				if (brakeLen > minMoveValue) {
					// 'brake' to end velocity
					ptLastEnd = addMoveSegment("brake", ptLastEnd, brakeAccel, brakeTime);
				}

				double totalTime = launchTime + coastTime + brakeTime;
				checkMoveTarget(ptLastEnd, targetPos);
				return totalTime;
			}

			if (endVel < minMoveValue && Math.abs(ptLastEnd.vel) < minMoveValue) {
				// short move bounded by acceleration
				launchLen = moveLen / 2;
				launchTime = Math.sqrt(2 * launchLen / maxAccel);
				brakeLen = launchLen;
				brakeTime = launchTime;
				// accelerate (or de-accel) to new velocity
				ptLastEnd = addMoveSegment("launch", ptLastEnd, launchAccel, launchTime);
				// 'brake' to end velocity
				ptLastEnd = addMoveSegment("brake ", ptLastEnd, brakeAccel, brakeTime);

				double totalTime = launchTime + brakeTime;
				checkMoveTarget(ptLastEnd, targetPos);
				return totalTime;
			}

			double aveVel = Math.abs(0.5 * (endVel * moveDirection + ptLastEnd.vel));
			if (aveVel > minMoveValue) {
				//
				launchTime = Math.abs(moveLen / aveVel);
				launchAccel = (endVel - ptLastEnd.vel) / launchTime;

				if (Math.abs(launchAccel) > maxAccel) {
					throw new AutoPilotException("Move path error: move accel (%.0f) exceeds max accel (%.0f)",
							Math.abs(launchAccel), maxAccel);
				}
				ptLastEnd = addMoveSegment("ramp", ptLastEnd, launchAccel, launchTime);
				checkMoveTarget(ptLastEnd, targetPos);
				return launchTime;
			}

			throw new AutoPilotException("Path planning failed from %.1f to %.1f", deltaPos, targetPos);
		}

		public MovePoint addMoveSegment(String desc, MovePoint ptLastEnd, double accel, double segmentTime) {
			MoveSegment msNew = new MoveSegment(desc, ptLastEnd, accel, segmentTime);
			return addMoveSegment(msNew);
		}

		public MovePoint addMoveSegment(MoveSegment msNew) {
			moveSegmentList.add(msNew);
			String msg = String.format("  %s %s", name, msNew.toString());
			traceMessage(msg);
			return msNew.ptEnd;
		}
	}

	// Movement between two states
	public class MoveSegment {
		public String desc;
		public double accel;
		public MovePoint ptStart;
		public MovePoint ptEnd;
		public MovePoint ptLast;

		public MoveSegment(String segmentDesc) {
			desc = segmentDesc;
			accel = 0;
			ptStart = new MovePoint();
			ptEnd = new MovePoint();
			ptLast = new MovePoint();
		}

		public MoveSegment(MoveSegment ms, double timeOffset, double distanceOffset) {
			desc = ms.desc;
			accel = ms.accel;
			ptStart = new MovePoint(ms.ptStart, timeOffset, distanceOffset);
			ptEnd = new MovePoint(ms.ptEnd, timeOffset, distanceOffset);
			ptLast = new MovePoint(ms.ptEnd, timeOffset, distanceOffset);
		}

		public MoveSegment(String segmentDesc, MovePoint ptLastEnd, double segmentAccel, double segmentSeconds) {
			desc = segmentDesc;
			accel = segmentAccel;
			ptStart = new MovePoint(ptLastEnd);
			ptEnd = new MovePoint();
			ptEnd.time = ptStart.time + segmentSeconds;
			ptEnd.vel = ptStart.vel + accel * segmentSeconds;
			ptEnd.pos = ptStart.pos + ptStart.vel * segmentSeconds + 0.5 * accel * segmentSeconds * segmentSeconds;
			ptLast = new MovePoint(ptEnd);
		}

		public MovePoint getPoint(double time) {
			if (time <= ptStart.time) {
				ptLast.set(ptStart);
			} else if (time >= ptEnd.time) {
				ptLast.set(ptEnd);
			} else {
				double dt = time - ptStart.time;
				ptLast.time = time;
				ptLast.vel = ptStart.vel + accel * dt;
				ptLast.pos = ptStart.pos + ptStart.vel * dt + 0.5 * accel * dt * dt;
			}
			return ptLast;
		}

		public String toString() {
			return String.format("%6.6s  T(%4.1f %4.1f)  A%4.0f  V(%6.1f %6.1f)  D(%6.1f %6.1f)", desc, ptStart.time,
					ptEnd.time, accel, ptStart.vel, ptEnd.vel, ptStart.pos, ptEnd.pos);
		}
	}

	// Movement state at given time
	public class MovePoint {
		public double time;
		public double vel;
		public double pos;

		public MovePoint() {
			time = 0;
			vel = 0;
			pos = 0;
		}

		public MovePoint(MovePoint mp, double timeOffset, double distanceOffset) {
			time = mp.time + timeOffset;
			vel = mp.vel;
			pos = mp.pos + distanceOffset;
		}

		public MovePoint(MovePoint pt) {
			time = pt.time;
			vel = pt.vel;
			pos = pt.pos;
		}

		public void set(MovePoint pt) {
			time = pt.time;
			vel = pt.vel;
			pos = pt.pos;
		}

		public String toString() {
			return String.format("T:%7.3f V:%7.1f P:%7.1f", time, vel, pos);
		}
	}

	@Retention(RetentionPolicy.RUNTIME)
	@Target(ElementType.METHOD)
	public @interface AutoPilotMethod {
		String argHint();
	}

	public class AutoPilotException extends Exception {

		private static final long serialVersionUID = -6982331850819279678L;

		public AutoPilotException(String msg) {
			super(msg);
		}

		public AutoPilotException(String fmt, Object... objects) {
			super(String.format(fmt, objects));
		}

		public AutoPilotException(String message, Throwable throwable) {
			super(message, throwable);
		}
	}

	// stores time, target pos/vel + actual pos/vel
	public class AutoPilotPerformance {
		private double seconds;
		private double[] targetPos;
		private double[] targetVel;
		private double[] actualPos;
		private double[] actualVel;

		public double seconds() {
			return seconds;
		}

		public AutoPilotPerformance(double s, double[] pos, double[] vel) {
			seconds = s;
			// target position
			if (pos != null) {
				targetPos = new double[] { pos[MOTOR_LEFT], pos[MOTOR_RIGHT] };
			} else {
				targetPos = new double[] { 0, 0 };
			}

			// target velocity
			if (vel != null) {
				targetVel = new double[] { vel[MOTOR_LEFT], vel[MOTOR_RIGHT] };
			} else {
				targetVel = new double[] { 0, 0 };
			}

			// actual distance
			actualPos = new double[] { Robot.driveTrainSRX.getLeftDistance(), Robot.driveTrainSRX.getRightDistance() };
			// actual velocity
			actualVel = new double[] { Robot.driveTrainSRX.getLeftVelocity(), Robot.driveTrainSRX.getRightVelocity() };
		}

		public String str() {
			return String.format("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%6.2f,%6.2f,%6.2f,%6.2f\n", seconds,
					// positions
					targetPos[MOTOR_LEFT], targetPos[MOTOR_RIGHT], actualPos[MOTOR_LEFT], actualPos[MOTOR_RIGHT],
					// velocities
					targetVel[MOTOR_LEFT], targetVel[MOTOR_RIGHT], actualVel[MOTOR_LEFT], actualVel[MOTOR_RIGHT]);
		}

	}

}
