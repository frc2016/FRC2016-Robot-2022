package org.usfirst.frc2016;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.BufferedReader;
import java.io.IOException;
import java.util.Collections;
import java.util.Objects;
import java.util.HashMap;
import java.util.Map;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc2016.commands.*;
import org.usfirst.frc2016.subsystems.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.ErrorCode;

public class PathPlanner {
    private String lastStatus;
    private Timer driveUpdateTimer;
    private int nDriveUpdateMs = 10;
    private String plansFolder = "/c/pathplanner";
    private String dashboardName = "PathStatus";
    private boolean isPathRunning = false;
    private ArrayList<PathPlan> robotPaths = new ArrayList<PathPlan>();
    private ArrayList<PathPerformance> robotPerf = new ArrayList<PathPerformance>();

    private long nanoTimeStart; // nanoseconds
    private PathPlan currentPath = null;
    private int lastPoseIndex = 0;

    final static int LEFT = 0;
    final static int RIGHT = 1;
    final static int PATH_AXES = 2;
    final static int FT_PER_IN = 12;

    // Motion Profile Support
    public enum MotionProfileState {
        Stopped, WaitingBuffer, Running
    }

    MotionProfileState mpState = MotionProfileState.Stopped;
    private boolean mpActive = false;
    Notifier mpNotifer = null;
    int mpUnderruns = 0;
    int mpProcessBuffers = 0;
    int mpShowStatus = 0;
    MotionProfileStatus[] mpStatus = new MotionProfileStatus[PATH_AXES];

    public PathPlanner() {
        // create axis array elements
        for (int axis = 0; axis < PATH_AXES; ++axis) {
            mpStatus[axis] = new MotionProfileStatus();
        }

        initializeRobotPaths();
    }

    ArrayList<PathPlan> robotPaths() {
        return robotPaths;
    }

    public PathPlan currentPath() {
        return currentPath;
    }

    // find the named path
    public PathPlan findPath(String name) {
        for (PathPlan path : robotPaths) {
            if (path.name().equals(name)) {
                return path;
            }
        }
        return null;
    }

    // read all the PathPlanner generated csv files into memory
    public void initializeRobotPaths() {
        nanoTimeStart = System.nanoTime();
        try {
            robotPaths = new ArrayList<PathPlan>();

            System.out.println("/- Initializing PathPlanner paths:");

            File folder = new File(plansFolder);
            for (File file : folder.listFiles()) {

                String filename = file.getName();
                String name = filename;
                String extension = "";
                int i = filename.lastIndexOf('.');
                if (i > 0) {
                    extension = filename.substring(i + 1);
                    name = filename.substring(0, i);
                }
                i = name.lastIndexOf('-');
                if (i > 0) {
                    name = filename.substring(0, i);
                }

                if (file.isFile() && extension.equalsIgnoreCase("csv")) {
                    robotPaths.add(new PathPlan(file, name));
                }
            }
            Collections.sort(robotPaths);

            PathPlan lastPath = null;
            for (PathPlan path : robotPaths) {
                path.loadFile();
                // System.out.printf("| %4d pts %5.1f secs: %s\n", path.points(),
                // path.runSeconds(), path.filePath());

                if (null != lastPath && lastPath.name().equalsIgnoreCase(path.name())) {
                    // extend last path with this one that has the same base name
                    lastPath.append(path);
                } else {
                    lastPath = path;
                }
            }

            // remove all the paths that extended others
            robotPaths.removeIf(p -> 0 == p.segments());

            System.out.println("|- Merged:");
            for (PathPlan path : robotPaths) {
                System.out.printf("| %4d pts %5.1f secs end %5.1f,%5.1f: %s\n", path.points(), path.runSeconds(),
                        path.endLeft(), path.endRight(), path.name());
            }

            lastStatus = String.format("\\- %d paths loaded in %.0f ms", robotPaths.size(), elapsedSeconds() * 1000);
        } catch (Exception e) {
            lastStatus = e.getMessage();
        }

        System.out.println(lastStatus);
        SmartDashboard.putString(dashboardName, lastStatus);
    }

    public void start(String pathName) {
        start(pathName, false);
    }

    // start running the named path
    public void start(String pathName, boolean useMP) {
        mpActive = useMP;
        mpUnderruns = 0;
        mpProcessBuffers = 0;
        mpShowStatus = -1;
        // get the game data
        // String rawGameData = DriverStation.getInstance().getGameSpecificMessage();

        isPathRunning = false;

        currentPath = findPath(pathName);
        if (null != currentPath) {
            robotPerf = new ArrayList<PathPerformance>();
            isPathRunning = true;
            lastPoseIndex = 0;
            Robot.driveTrainSRX.resetEncoders();

            mpState = MotionProfileState.Stopped;
            if (mpActive) {
                Robot.driveTrainSRX.motionProfileSetValue(SetValueMotionProfile.Disable);
                currentPath.mpStartFilling();
                mpState = MotionProfileState.WaitingBuffer;
                System.out.println("mpState WaitingBuffer");
                mpPrintStatus(true);
            }

            nanoTimeStart = System.nanoTime();
            startTimerTask();

            SmartDashboard.putString(dashboardName, "Running " + pathName);
        } else {
            SmartDashboard.putString(dashboardName, "path missing " + pathName);
        }
    }

    // Called repeatedly from a 'Command'
    public void execute() {
        logRobotPref();
        if (mpActive) {
            mpCheckState();
        }
        Robot.driveTrainSRX.pingDifferentialDrive();
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return !isPathRunning;
    }

    public boolean isRunning() {
        return isPathRunning;
    }

    // Called once after isFinished returns true
    public void end() {
        shutdown("end");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    // --@Override
    public void interrupted() {
        shutdown("interrupted");
    }

    // stop running the path, stopping the robot
    public void shutdown(String desc) {
        isPathRunning = false;
        mpState = MotionProfileState.Stopped;
        Robot.driveTrainSRX.motionProfileSetValue(SetValueMotionProfile.Disable);
        System.out.println("Shutdown " + desc);
        mpPrintStatus(true);

        killTimerTask();

        // stop all the robot parts
        Robot.driveTrainSRX.driveStop();
        Robot.driveTrainSRX.motionProfileStop();

        Robot.ballIntake.stop();

        logSave();
    }

    // seconds from beginning of the move
    public double elapsedSeconds() {
        return (double) (System.nanoTime() - nanoTimeStart) / 1e9;
    }

    TalonFX mpTalon(int axis) {
        TalonFX talon = null;
        switch (axis) {

            case LEFT:
                talon = Robot.driveTrainSRX.getTalonFXLeft1();
                break;

            case RIGHT:
                talon = Robot.driveTrainSRX.getTalonFXRight1();
                break;
        }
        return talon;
    }

    boolean mpGetStatus(int axis) {
        TalonFX talon = mpTalon(axis);
        if (talon.getControlMode().value != TalonFXControlMode.MotionProfile.value) {
            // not in motion profile mode, exit
            shutdown("control mode not MP");
            return false;
        }
        MotionProfileStatus s = mpStatus[axis];
        talon.getMotionProfileStatus(s);
        if (s.hasUnderrun) {
            ++mpUnderruns;
            if (0 == (mpUnderruns % 100)) {
                double ups = mpProcessBuffers / elapsedSeconds();
                System.out.printf(" %d underruns %.0f processed/sec\n", mpUnderruns, ups);
            }
            talon.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);
        }
        return true;
    }

    void mpCheckState() {

        switch (mpState) {

            case Stopped:
                shutdown("mpState == Stopped");
                break;

            case WaitingBuffer: {
                // minimum points buffered in talon prior to starting
                int kMinPointsInTalon = 50;

                // check all the axis to set if they are ready
                boolean ready = true;
                for (int axis = 0; axis < PATH_AXES; ++axis) {
                    ready &= mpGetStatus(axis);
                    ready &= (mpStatus[axis].btmBufferCnt > kMinPointsInTalon);
                }

                if (ready) {

                    System.out.printf("Falcons have %d+%d points\n", mpStatus[0].btmBufferCnt,
                            mpStatus[1].btmBufferCnt);

                    // all ready, start them 'together'
                    Robot.driveTrainSRX.motionProfileSetValue(SetValueMotionProfile.Enable);
                    mpPrintStatus(true);

                    mpState = MotionProfileState.Running;
                    System.out.println("mpState running");
                }
                break;
            }

            case Running: {
                boolean done = true;
                for (int axis = 0; axis < PATH_AXES; ++axis) {
                    done &= mpGetStatus(axis);
                    done &= (mpStatus[axis].activePointValid && mpStatus[axis].isLast);
                }

                if (done) {
                    shutdown("running done");
                }

                double elapsed = elapsedSeconds();
                int rpt = (int) (elapsed / .25);
                if (mpShowStatus != rpt) {
                    mpShowStatus = rpt;
                    mpPrintStatus(false);
                }
                break;
            }
        }

    }

    void mpPrintStatus(boolean update) {
        if (update) {
            for (int axis = 0; axis < PATH_AXES; ++axis) {
                mpGetStatus(axis);
            }
        }
        MotionProfileStatus l = mpStatus[LEFT];
        MotionProfileStatus r = mpStatus[RIGHT];

        StringBuilder sb = new StringBuilder();
        sb.append(String.format("%.3f ", elapsedSeconds()));
        sb.append(String.format("btm:%d,%d ", l.btmBufferCnt, r.btmBufferCnt));
        // sb.append(String.format("rem:%d,%d ", l.topBufferRem, r.topBufferRem));
        sb.append(String.format("cnt:%d,%d ", l.topBufferCnt, r.topBufferCnt));
        sb.append(bfmt("has:", l.hasUnderrun, r.hasUnderrun));
        sb.append(bfmt("isu:", l.isUnderrun, r.isUnderrun));
        sb.append(bfmt("av:", l.activePointValid, r.activePointValid));
        sb.append(bfmt("lst:", l.isLast, r.isLast));
        sb.append(String.format("oe:%d,%d ", (int) l.outputEnable.value, (int) r.outputEnable.value));
        sb.append(String.format("ms:%d,%d", l.timeDurMs, r.timeDurMs));
        System.out.println(sb.toString());
    }

    String bfmt(String desc, boolean l, boolean r) {
        String f = desc + (l ? "1" : "0") + (r ? "1" : "0") + " ";
        return f;
    }

    // period task to move points from RIO queue to the Falcon queue
    class MotionProfileProcessBuffer implements java.lang.Runnable {
        public void run() {
            if ((++mpProcessBuffers & 1) == 1) {
                Robot.driveTrainSRX.getTalonFXLeft1().processMotionProfileBuffer();
                Robot.driveTrainSRX.getTalonFXRight1().processMotionProfileBuffer();
            } else {
                Robot.driveTrainSRX.getTalonFXRight1().processMotionProfileBuffer();
                Robot.driveTrainSRX.getTalonFXLeft1().processMotionProfileBuffer();
            }
        }
    }

    // stops the timer task for RIO updates of positions
    private void killTimerTask() {
        if (null != driveUpdateTimer) {
            driveUpdateTimer.cancel();
            driveUpdateTimer.purge();
            driveUpdateTimer = null;
        }
        if (null != mpNotifer) {
            mpNotifer.close();
            mpNotifer = null;
        }
    }

    // start the timer task for RIO updates of positions
    private void startTimerTask() {
        killTimerTask();
        if (mpActive) {
            mpNotifer = new Notifier(new MotionProfileProcessBuffer());
            int updateMs = currentPath.intervalMs() / 2;
            mpNotifer.startPeriodic(0.001 * updateMs);

        } else {
            driveUpdateTimer = new Timer();
            driveUpdateTimer.scheduleAtFixedRate(new DriveTimerTask(), nDriveUpdateMs, nDriveUpdateMs);
        }
    }

    // task exexuted every timer tick to update the drive train position
    class DriveTimerTask extends TimerTask {
        @Override
        public void run() {
            if (isFinished() || currentPath == null) {
                killTimerTask();
            } else {
                PathPose pnt = null;
                double seconds = elapsedSeconds();
                if (seconds >= currentPath.runSeconds()) {
                    // time is up, move to last point
                    isPathRunning = false;
                    pnt = currentPath.endPose();
                } else {
                    // get position for current time
                    lastPoseIndex = currentPath.findPoseIndex(lastPoseIndex, seconds);
                    pnt = currentPath.interpolate(lastPoseIndex, seconds);
                }
                if (null != pnt) {

                    Robot.driveTrainSRX.goToDistance(pnt.posLeft(), pnt.posRight());

                    // autoPilot.traceMessage("Cmd %.1f %.1f Pos %.1f %.1f ",
                    // driveDistances[AutoPilot.MOTOR_LEFT],
                    // driveDistances[AutoPilot.MOTOR_RIGHT], Robot.driveTrainSRX.getLeftDistance(),
                    // Robot.driveTrainSRX.getRightDistance());
                } else {
                    shutdown("path end");
                }
            }
        }
    }

    // log an entry into the robot performance log
    private void logRobotPref() {
        if (currentPath == null) {
            return;
        }
        double seconds = elapsedSeconds();
        int index = currentPath.findPoseIndex(lastPoseIndex, seconds);
        PathPose pnt = currentPath.interpolate(index, seconds);
        if (pnt == null) {
            return;
        }

        robotPerf.add(new PathPerformance(seconds, pnt));
    }

    private void logSave() {
        if (currentPath != null && robotPerf.size() > 0) {
            String msg = "";
            try {
                String filename = "/tmp/" + currentPath.name() + "_log.csv";
                FileWriter fw = new FileWriter(filename);

                String header = "time,tpl,tpr,apl,apr,tvl,tvr,avl,avr\n";
                fw.write(header);

                int index = 0;
                PathPerformance lastPerf = null;
                for (PathPerformance p : robotPerf) {

                    // write line into log
                    fw.write(p.str(lastPerf));
                    lastPerf = p;

                    // ping the drive to keep the watchdog happy
                    if (++index % 100 == 0) {
                        Robot.driveTrainSRX.pingDifferentialDrive();
                    }
                }

                fw.close();
                msg = String.format("%s end @ %3.1f %d pts", filename, elapsedSeconds(), robotPerf.size());

            } catch (Exception e) {
                msg = "RobotPerf save error: " + e.getMessage();
            }

            robotPerf = new ArrayList<PathPerformance>();
            System.out.println(msg);
            SmartDashboard.putString(dashboardName, "stopped " + msg);
        }
    }

    // a named path from imported from PathPlanner
    public class PathPlan implements Comparable<PathPlan> {
        private String filePath;
        private String name;
        private double runSeconds = 0;
        private int segments = 0;
        private ArrayList<PathPose> poseList;
        private BufferedTrajectoryPointStream[] ptStreams = new BufferedTrajectoryPointStream[PATH_AXES];

        public PathPlan(File file, String fileroot) {
            filePath = file.getPath();
            name = fileroot;
            poseList = new ArrayList<PathPose>();
        }

        public String filePath() {
            return filePath;
        }

        public String name() {
            return name;
        }

        public double runSeconds() {
            return runSeconds;
        }

        public int points() {
            return poseList.size();
        }

        public int segments() {
            return segments;
        }

        public double endLeft() {
            return poseList.size() > 0 ? poseList.get(poseList.size() - 1).posLeft() : 99.9;
        }

        public double endRight() {
            return poseList.size() > 0 ? poseList.get(poseList.size() - 1).posRight() : 99.9;
        }

        void loadFile() throws Exception {

            poseList = new ArrayList<PathPose>();
            runSeconds = 0;
            int lineNumber = 0;
            BufferedReader inputReader = null;

            String[] colNames = new String[] { "t", "pl", "pr", "vl", "vr", "h" };
            int[] colIndex = new int[colNames.length];

            try {
                inputReader = new BufferedReader(new FileReader(filePath));

                String fileLine;
                while (null != (fileLine = inputReader.readLine())) {
                    ++lineNumber;

                    String[] data = fileLine.split(",", 0);

                    if (lineNumber == 1) {
                        // csv header line: names of columns

                        // determine column of each required field
                        for (int col = 0; col < colNames.length; col++) {

                            // look for column name in the data
                            int fieldCol = -1;
                            for (int i = 0; i < data.length; ++i) {
                                if (colNames[col].equals(data[i])) {
                                    fieldCol = i;
                                    break;
                                }
                            }

                            if (fieldCol < 0) {
                                String msg = String.format("Column '%s' missing in csv file", col, data[col],
                                        colNames[col]);
                                throw new Exception(msg);
                            }

                            colIndex[col] = fieldCol;
                        }
                    } else {

                        // covert strings to doubles
                        Double[] d = new Double[data.length];
                        for (int col = 0; col < data.length; ++col) {
                            d[col] = Double.parseDouble(data[col]);
                        }

                        double sec = d[colIndex[0]];
                        double pl = d[colIndex[1]];
                        double pr = d[colIndex[2]];
                        double vl = d[colIndex[3]];
                        double vr = d[colIndex[4]];
                        double h = d[colIndex[5]];

                        poseList.add(new PathPose(sec, pl, pr, vl, vr, h));

                        runSeconds = sec;
                    }
                }

                segments = 1;

            } catch (Exception e) {
                String msg = String.format("Error Line %d: %s", lineNumber, e.getMessage());
                throw new Exception(msg);
            }

            if (inputReader != null) {
                inputReader.close();
            }

            segments = poseList.size() > 0 ? 1 : 0;
        }

        public PathPose endPose() {
            int size = poseList.size();
            return size > 0 ? poseList.get(size - 1) : null;
        }

        // append antoher path to the end of this one
        public void append(PathPlan pathNext) {

            PathPose poseEnd = new PathPose();
            if (poseList.size() > 0) {
                poseEnd = poseList.get(poseList.size() - 1);
            }

            for (PathPose poseAdd : pathNext.poseList) {
                poseList.add(poseAdd.offset(poseEnd));
            }

            segments += pathNext.segments;
            runSeconds += pathNext.runSeconds;

            pathNext.segments = 0;
            pathNext.runSeconds = 0;
        }

        // return index before 'seconds', next index is after 'seconds'
        public int findPoseIndex(int lastIndex, double seconds) {
            for (int index = lastIndex + 1; index < poseList.size(); ++index) {
                if (poseList.get(index).seconds() >= seconds) {
                    // pose is in the future, return one prior
                    return lastIndex;
                }
                // pose in in the past, advance
                lastIndex = index;
            }
            // pose is after last item;
            return poseList.size();
        }

        public PathPose interpolate(int index, double seconds) {
            int size = poseList.size();
            if (index >= size) {
                return null;
            }

            PathPose poseLast = poseList.get(index);
            index++;
            if (index >= size) {
                return poseLast;
            }

            PathPose poseNext = poseList.get(index);
            return poseLast.interpolate(poseNext, seconds);
        }

        public void initStreams() {
            int durationMs = 10;
            if (poseList.size() > 1)
                durationMs = (int) (1000 * (poseList.get(1).seconds() - poseList.get(1).seconds()));

            for (int axis = 0; axis < PATH_AXES; ++axis) {
                ptStreams[axis] = new BufferedTrajectoryPointStream();

                TrajectoryPoint pt = new TrajectoryPoint();
                for (int index = 0; index < poseList.size(); index++) {

                    PathPose pose = poseList.get(index);
                    pose.setTrajectoryPoint(pt, axis, durationMs);

                    pt.zeroPos = (index == 0);
                    pt.isLastPoint = (index == (poseList.size() - 1));

                    ptStreams[axis].Write(pt);
                }
            }
        }

        int intervalMs() {
            int ms = 20;
            if (poseList.size() > 1) {
                ms = (int) (1000 * (poseList.get(1).seconds() - poseList.get(0).seconds()));
            }
            return ms;
        }

        void mpStartFilling() {
            int durationMs = intervalMs();
            System.out.printf("mpStart Filling duration %d ms  %d pts\n", durationMs, poseList.size());
            mpWriteProfile(LEFT, Robot.driveTrainSRX.getTalonFXLeft1(), durationMs);
            mpWriteProfile(RIGHT, Robot.driveTrainSRX.getTalonFXRight1(), durationMs);
        }

        void mpWriteProfile(int axis, TalonFX talon, int durationMs) {
            talon.clearMotionProfileHasUnderrun(Constants.kTimeoutMs);

            // just in case we are interrupting another MP and there is still buffer
            // points in memory, clear it.

            talon.clearMotionProfileTrajectories();

            // set the base trajectory period to zero, use the individual trajectory period
            // below
            // talon.configMotionProfileTrajectoryPeriod(durationMs, Constants.kTimeoutMs);
            talon.configMotionProfileTrajectoryPeriod(Constants.kBaseTrajPeriodMs, Constants.kTimeoutMs);

            // set to half of point intervals
            talon.changeMotionControlFramePeriod(durationMs / 2);

            TrajectoryPoint pt = new TrajectoryPoint();
            for (int index = 0; index < poseList.size(); index++) {

                PathPose pose = poseList.get(index);
                // pt = pose.ToTrajectoryPoint(axis, 0);
                pose.setTrajectoryPoint(pt, axis, durationMs);

                pt.zeroPos = (index == 0);
                pt.isLastPoint = (index == (poseList.size() - 1));

                talon.pushMotionProfileTrajectory(pt);
            }
        }

        @Override
        public int compareTo(PathPlan other) {
            return filePath.compareTo(other.filePath);
        }
    }

    // a single point in the path plan
    public class PathPose {
        private double seconds = 0;
        private double[] pos = new double[PATH_AXES]; // inches
        private double[] vel = new double[PATH_AXES]; // inches/sec
        private double heading = 0; // heading in degress

        double seconds() {
            return seconds;
        }

        double posLeft() {
            return pos[LEFT];
        }

        double posRight() {
            return pos[RIGHT];
        }

        double[] pos() {
            return pos;
        }

        double[] vel() {
            return vel;
        }

        double heading() {
            return heading;
        }

        public PathPose() {
        }

        // ctor for PathPlanner files. Positions in ft, vel in ft/sec
        public PathPose(double sec, double pl, double pr, double vl, double vr, double h) {
            seconds = sec;
            // convert to inches
            pos[LEFT] = pl * FT_PER_IN;
            pos[RIGHT] = pr * FT_PER_IN;
            vel[LEFT] = vl * FT_PER_IN;
            vel[RIGHT] = vr * FT_PER_IN;
            heading = h;
        }

        // copy constructor
        public PathPose(PathPose r) {
            seconds = r.seconds;
            // deep copy, allow indepent changes to vector elements
            for (int axis = 0; axis < PATH_AXES; ++axis) {
                pos[axis] = r.pos[axis];
                vel[axis] = r.vel[axis];
            }
            heading = r.heading;
        }

        public PathPose offset(PathPose end) {
            PathPose sum = new PathPose(this);
            // only add the time and position from the end point of the last segment
            sum.seconds += end.seconds;
            for (int axis = 0; axis < PATH_AXES; ++axis) {
                sum.pos[axis] += end.pos[axis];
            }
            return sum;
        }

        public PathPose interpolate(PathPose next, double sec) {
            if (sec <= seconds) {
                // target is less than first point (this)
                return this;
            }
            if (sec >= next.seconds) {
                // target time is greate than next point
                return next;
            }

            PathPose mid = new PathPose(this);
            double f2 = (sec - seconds) / (next.seconds - seconds);
            double f1 = 1 - f2;
            mid.seconds = sec;
            for (int axis = 0; axis < PATH_AXES; ++axis) {
                mid.pos[axis] += f1 * pos[axis] + f2 * next.pos[axis];
                mid.vel[axis] += f1 * vel[axis] + f2 * next.vel[axis];
            }
            return mid;
        }

        void setTrajectoryPoint(TrajectoryPoint point, int axis, int durationMilliseconds) {
            // for each point, fill Falcon structure
            point.timeDur = durationMilliseconds;
            point.position = pos[axis] * DriveTrainSRX.COUNTS_PER_INCH; // encoder counts
            point.velocity = vel[axis] * DriveTrainSRX.SPEED_SCALE; // counts/100ms
            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = Constants.kSlot_MotProf; // which set of gains would you like to use [0,3]?
            point.profileSlotSelect1 = 0; // auxiliary PID [0,1], leave zero
            point.zeroPos = false; // set this to true on the first point
            point.isLastPoint = false; // set this to true on the last point
            point.arbFeedFwd = 0; // you can add a constant offset to add to PID[0] output here
        }

    }

    // stores time, target pos/vel + actual pos/vel
    public class PathPerformance {
        private double seconds;
        private double[] targetPos;
        private double[] targetVel;
        private double[] actualPos;
        private double[] actualVel = new double[PATH_AXES];

        public double seconds() {
            return seconds;
        }

        public PathPerformance(double s, PathPose p) {
            seconds = s;
            targetPos = p.pos;
            targetVel = p.vel;

            // actual distance
            actualPos = new double[] { Robot.driveTrainSRX.getLeftDistance(), Robot.driveTrainSRX.getRightDistance() };
            // actual velocity
            actualVel = new double[] { Robot.driveTrainSRX.getLeftVelocity(), Robot.driveTrainSRX.getRightVelocity() };
        }

        public String str(PathPerformance last) {
            double[] tv = new double[] { 0, 0 };
            if (last != null) {
                double dt = seconds - last.seconds;
                tv[LEFT] = (targetPos[LEFT] - last.targetPos[LEFT]) / dt;
                tv[RIGHT] = (targetPos[RIGHT] - last.targetPos[RIGHT]) / dt;
            }

            return String.format("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%6.2f,%6.2f,%6.2f,%6.2f\n", seconds,
                    // positions
                    targetPos[LEFT], targetPos[RIGHT], actualPos[LEFT], actualPos[RIGHT],
                    // velocities
                    targetVel[LEFT], targetVel[RIGHT], actualVel[LEFT], actualVel[RIGHT]);
        }

    }

}
