package org.usfirst.frc2016.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Random;

import javax.sound.sampled.SourceDataLine;

import org.usfirst.frc2016.Config;
import org.usfirst.frc2016.Robot;

public abstract class MotorBase {
    protected String configName; // base name of parameters in configuration file
    protected String instanceName; // base name of parameters in configuration file
    protected String name;
    protected int deviceId; // can bus device id
    protected int followId; // non-zero -> this motor follows another
    protected double countsPerUnit = 1;
    protected double speedScale = 1;
    protected String unitsName = "counts";
    protected double pid_P = 0;
    protected double pid_I = 0;
    protected double pid_D = 0;
    protected double pid_F = 0;
    protected double pid_IZ = 0;
    protected double moveVel = 30; // units/sec
    protected double moveAccel = 30; // units/sec*sec
    protected double ampMax = 200;

    // motor varaibles are updated by updateMotorStats
    protected double motorEncoder = 0; // motor position in native controller units
    protected double motorPos = 0; // units
    protected double motorPosErr = 0; // units
    protected double motorVel = 0; // units/sec
    protected double motorPwr = 0; // -1 to 1, last set power

    protected double targetPos = 0; // units
    protected double targetVel = 0; // units/sec

    protected int countsPerRev = 1024;
    protected int maxRevsPerSec = 5000;

    protected String fmtEnc;
    protected String fmtPos;
    protected String fmtErr;
    protected String fmtVel;

    static ArrayList<MotorBase> allMotors = new ArrayList<>();

    public MotorBase(String configName, String instanceName, int deviceId, int followId) {
        this.configName = configName;
        this.instanceName = instanceName;
        this.name = configName;
        if (instanceName != null && instanceName.length() > 0) {
            name = configName + "_" + instanceName;
        }

        this.deviceId = deviceId;
        this.followId = followId;
        setDisplayFormats(0, 1, 1, 0);
        allMotors.add(this);
    }

    // sets numbner of decimal places for the of the motor stats in smart dashboard
    public void setDisplayFormats(int enc, int pos, int posErr, int vel) {
        fmtEnc = String.format("%%.%df", enc);
        fmtPos = String.format("%%.%df", pos);
        fmtErr = String.format("%%.%df", posErr);
        fmtVel = String.format("%%.%df", vel);
       // System.out.println("Fmts " + fmtEnc + " " + fmtPos + " " + fmtErr + " " + fmtVel);
    }

    // update all motors, called from Robot.Periodic
    public static void updateAll() {
        for (MotorBase m : allMotors) {
            m.updateMotorStats();
        }
    }

    // show stats of all motors,
    public static void displayAll() {
        for (MotorBase m : allMotors) {
            if (m.followId <= 0) {
                SmartDashboard.putString(m.name + " enc", String.format(m.fmtEnc, m.motorEncoder));
                SmartDashboard.putString(m.name + " pos", String.format(m.fmtPos, m.motorPos));
                SmartDashboard.putString(m.name + " err", String.format(m.fmtErr, m.motorPosErr));
                SmartDashboard.putString(m.name + " vel", String.format(m.fmtVel, m.motorVel));
            }
        }
    }

    public void setUnitsNative() {
        this.countsPerUnit = 1;
        this.speedScale = 1;
    }

    public void setUnits(String unitsName, double cntPerUnit) {
        this.unitsName = unitsName;
        this.countsPerUnit = cntPerUnit;
    }

    public String getName() {
        return name;
    }

    public String getConfigName() {
        return configName;
    }

    public String getInstanceName() {
        return instanceName;
    }

    public int getCntsPerRev() {
        return countsPerRev;
    }

    public int getMaxRevsPerSec() {
        return maxRevsPerSec;
    }

    // motor position in natove encoder units (normally counts)
    public double getMotorEncoder() {
        return motorEncoder;
    }

    // motor position scaled to user units (ex. inches)
    public double getMotorPos() {
        return motorPos;
    }

    // motor velocity scaled to user units (ex. inches/sec)
    public double getMotorVel() {
        return motorVel;
    }

    public double getMotorPwr() {
        return motorPwr;
    }

    public double getTargetPos() {
        return targetPos;
    }

    public double getMoveVel() {
        return moveVel;
    }

    public double getMoveAccel() {
        return moveAccel;
    }

    // enable limit switches
    public abstract void enableLimitSwitches(boolean fwdEnable, boolean revEnable);

    // set idle mode
    public abstract void enableIdleBrake(boolean enableBrake);

    // set max current
    public abstract void setCurrentLimit(boolean enable, double ampMax);

    // update state variables from the motor
    public abstract void updateMotorStats();

    // stop motion
    public abstract void stop();

    // zero the motor's position
    public abstract void zeroMotorPos();

    // set closed loop controller to targetPos (units)
    public abstract void snapTo(double pos);

    // move closed loop controller by deltaPos (units)
    public void snapInc(double deltaPos) {
        snapTo(targetPos + deltaPos);
    }

    // move to pos using vel and acc constraints
    public abstract void moveTo(double pos, double vel, double acc);

    // move to pos using moveVel and moveAcell
    public void moveTo(double pos) {
        moveTo(pos, moveVel, moveAccel);
    }

    // move to targetPos+deltaPos using current cruiseVel and accel constraints
    public void moveInc(double deltaPos) {
        moveTo(targetPos + deltaPos);
    }

    // start moving at velocity (units)
    public abstract void moveByPower(double fraction);

    // start moving at voltagle (-1 -> 1)
    public abstract void moveByVoltage(double fraction);

    private double getLowerPower(double stopSeconds) {
        double lowerPower = motorPwr;
        if (motorPwr > 0) {
            lowerPower = Math.max(0.0, motorPwr - getMoveVel() * Robot.kDefaultPeriod / stopSeconds);
        } else {
            lowerPower = Math.min(0.0, motorPwr + getMoveVel() * Robot.kDefaultPeriod / stopSeconds);
        }
        return lowerPower;
    }

    public void slowerPower(double stopSeconds) {
        moveByPower(getLowerPower(stopSeconds));
    }

    //
    public void slowerVoltage(double stopSeconds) {
        moveByVoltage(getLowerPower(stopSeconds));
    }

    public void loadConfig() {
        Config cfg = Robot.config;
        pid_P = cfg.getDouble(configName + "_P", 0);
        pid_I = cfg.getDouble(configName + "_I", 0);
        pid_D = cfg.getDouble(configName + "_D", 0);
        pid_F = cfg.getDouble(configName + "_F", 0);
        pid_IZ = cfg.getDouble(configName + "_IZ", 0);
        moveVel = cfg.getDouble(configName + "_vel", 40);
        moveAccel = cfg.getDouble(configName + "_acc", 40);
        ampMax = cfg.getDouble(configName + "_ampMax", 40);
    }

   
}
