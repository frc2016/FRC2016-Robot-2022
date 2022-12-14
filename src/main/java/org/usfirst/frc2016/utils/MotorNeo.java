package org.usfirst.frc2016.utils;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax.IdleMode;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

public class MotorNeo extends MotorBase {
    private CANSparkMax neo;
    private SparkMaxPIDController neoPid;
    private RelativeEncoder neoEncoder;
    private SparkMaxLimitSwitch neoRevLimit;
    private SparkMaxLimitSwitch neoFwdLimit;

    private static final int FREE_RPM = 5000; // Neo 500
    private final double MAX_VOLTAGE = 11;

    private final int smartMotionSlot = 0;

    // when the user units are "inches"
    double revsPerUnit = 1;  // revolutions <neo unit> = revsPerUnit * postion ("inches")
    double rpmPerUps = 1; // rpm <neo unit> = rpmPerUps * velocity ("inches/sec")

    CANSparkMax getNeo() {
        return neo;
    }

    public MotorNeo(String configName, String instanceName, int devId, int followId, boolean inverted) {
        super(configName, instanceName, devId, followId);

        maxRevsPerSec = FREE_RPM / 60;

        loadConfig();

        neo = new CANSparkMax(devId, MotorType.kBrushless);

        initialize(inverted, followId);
    }

    @Override
    public void setUnitsNative() {
        super.setUnitsNative();
        revsPerUnit = 1;
        rpmPerUps = 1;
    }

    @Override
    public void setUnits(String unitsName, double cntsPerUnit) {
        super.setUnits(unitsName, cntsPerUnit);
        // native scale for position control is in revolutions
        revsPerUnit = countsPerUnit / countsPerRev; 
        // native scale for velocity control is RPM
        // rpm = rpmPerUps * velocity (in user units/sec)
        rpmPerUps = revsPerUnit / 60;
    }

    void initialize(boolean inverted, int followId) {
        neoPid = neo.getPIDController();
        neoEncoder = neo.getEncoder();
        neoFwdLimit = neo.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        neoRevLimit = neo.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        countsPerRev = neoEncoder.getCountsPerRevolution();

        neo.clearFaults();
        neo.restoreFactoryDefaults();

        neo.setInverted(inverted);
        enableIdleBrake(true);

        neo.enableVoltageCompensation(MAX_VOLTAGE);

        neo.enableSoftLimit(SoftLimitDirection.kForward, false);
        neo.enableSoftLimit(SoftLimitDirection.kReverse, false);
        enableLimitSwitches(false, false);

        neoPid.setP(pid_P);
        neoPid.setI(pid_I);
        neoPid.setD(pid_D);
        neoPid.setIZone(pid_IZ);
        neoPid.setFF(pid_I);
        neoPid.setOutputRange(-1, 1);

        if (ampMax > 0) {
            setCurrentLimit(true, ampMax);
        }

        if (followId > 0) {
            // lookup the master in the list of defined motors
            for (MotorBase motorBase : allMotors) {
                if (motorBase.deviceId == followId && motorBase instanceof MotorNeo) {
                    MotorNeo followMotor = (MotorNeo) motorBase;
                    neo.follow(followMotor.neo);
                }
            }
        }

        neo.clearFaults();
    }

    // override base class methods
    @Override
    // enable hardware limit switches
    public void enableLimitSwitches(boolean fwdEnable, boolean revEnable) {
        neoFwdLimit.enableLimitSwitch(fwdEnable);
        neoRevLimit.enableLimitSwitch(revEnable);
    }

    @Override
    // set idle mode
    public void enableIdleBrake(boolean enableBrake) {
        neo.setIdleMode(enableBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    // set max current
    public void setCurrentLimit(boolean enable, double amps) {
        if (!enable)
            amps = 200;
        neo.setSmartCurrentLimit((int) amps);
    }

    @Override
    public void updateMotorStats() {
        motorEncoder =  neoEncoder.getPosition();
        motorPos = motorEncoder / revsPerUnit;
        motorPosErr = 0; // ?
        motorVel = neoEncoder.getVelocity() / rpmPerUps;
    }

    @Override
    // stop motion
    public void stop() {
        neo.set(0);
        motorPwr = 0;
    }

    @Override
    // update state variables from the motor
    public void zeroMotorPos() {
        neoEncoder.setPosition(0);
        motorPos = 0;
        targetPos = 0;
    }

    // set closed loop controller to targetPos (units)
    @Override
    public void snapTo(double pos) {
        neoPid.setReference(pos * revsPerUnit, CANSparkMax.ControlType.kPosition);
        this.targetPos = pos;
    }

    // move to targetPos using current cruiseVel and accel constraints
    @Override
    public void moveTo(double pos, double vel, double acc) {
        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        neoPid.setSmartMotionMaxVelocity(vel * rpmPerUps, smartMotionSlot);
        neoPid.setSmartMotionMinOutputVelocity(vel * rpmPerUps / 10, smartMotionSlot);
        neoPid.setSmartMotionMaxAccel(acc * rpmPerUps, smartMotionSlot);

        /**
         * As with other PID modes, Smart Motion is set by calling the
         * setReference method on an existing pid object and setting
         * the control type to kSmartMotion
         */
        neoPid.setReference(pos * revsPerUnit, CANSparkMax.ControlType.kSmartMotion);
        targetPos = pos;
    }

    @Override
    public void moveTo(double pos) {
        neoPid.setReference(pos * revsPerUnit, CANSparkMax.ControlType.kSmartMotion);
        targetPos = pos;
    }

    // start moving by power factor (-1 -> 1)
    @Override
    public void moveByPower(double fraction) {
        neo.set(fraction);
        motorPwr = fraction;
    }

    // start moving at voltage (-1 -> 1)
    @Override
    public void moveByVoltage(double fraction) {
        neo.setVoltage(fraction * MAX_VOLTAGE);
        motorPwr = fraction;
    }
}
