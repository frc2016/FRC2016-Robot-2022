package org.usfirst.frc2016.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
//import com.ctre.phoenix.motion.SetValueMotionProfile;
//import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class MotorFalcon extends MotorBase {
    private WPI_TalonFX talon;

    private final int pidSlot0 = 0;
    private final int primaryController = 0;
    private final int timeoutMs = 0;
    private static final double SEC_PER_100MS = 0.1; // falcon vel units are cnts/100ms
    private static final int CNT_PER_REV = 2048; // Falcon 500
    private static final int FREE_RPM = 6380; // Falcon 500

    WPI_TalonFX getTalon() {
        return talon;
    }

    public MotorFalcon(String configName, String instanceName, int devId, int followId, boolean inverted) {
        super(configName, instanceName, devId, followId);

        countsPerRev = CNT_PER_REV;
        maxRevsPerSec = FREE_RPM / 60;

        loadConfig();

        talon = new WPI_TalonFX(devId);

        initialize(inverted, followId);
    }

    @Override
    public void setUnitsNative() {
        super.setUnitsNative();
        countsPerUnit = 1;
        speedScale = 1;
    }

    @Override
    public void setUnits(String unitsName, double cntsPerUnit) {
        super.setUnits(unitsName, cntsPerUnit);
        speedScale = super.countsPerUnit * SEC_PER_100MS;
    }

    void initialize(boolean inverted, int followId) {
        stop();

        talon.configFactoryDefault();
        talon.clearStickyFaults(0);

        talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, pidSlot0, timeoutMs);
        talon.setSensorPhase(true); // !!!! Check this !!!!!
        talon.setInverted(inverted);

        enableIdleBrake(true);
        zeroMotorPos();

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen, timeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen, timeoutMs);

        enableLimitSwitches(false, false);

        talon.configAllowableClosedloopError(pidSlot0, 0, timeoutMs);
        talon.setIntegralAccumulator(0, pidSlot0, timeoutMs);
        talon.config_kP(pidSlot0, pid_P, timeoutMs);
        talon.config_kI(pidSlot0, pid_I, timeoutMs);
        talon.config_kD(pidSlot0, pid_D, timeoutMs);
        talon.config_kF(pidSlot0, pid_F, timeoutMs);

        // limit current on the stator, slows down motor heating
        StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(
                false, // disable limit
                20, // limited to amps
                25, // when over trigger threshold
                1.0); // for seconds
        talon.configStatorCurrentLimit(statorLimit);

        if (ampMax > 0) {
            setCurrentLimit(true, ampMax);
        }

        talon.configNominalOutputForward(0, 0);
        talon.configNominalOutputReverse(0, 0);
        talon.configPeakOutputForward(1, 0);
        talon.configPeakOutputReverse(-1, 0);

        if (followId > 0) {
            talon.set(ControlMode.Follower, followId);
        } else {
            // set update rate for reading values
            talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeoutMs);
            talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutMs);
        }
    }

    // override base class methods
    @Override
    // enable hardware limit switches
    public void enableLimitSwitches(boolean fwdEnable, boolean revEnable) {
        talon.configForwardSoftLimitEnable(fwdEnable, timeoutMs);
        talon.configReverseSoftLimitEnable(revEnable, timeoutMs);
        // talon.overrideLimitSwitchesEnable(!enable);
    }

    @Override
    // set idle mode
    public void enableIdleBrake(boolean enableBrake) {
        talon.setNeutralMode(enableBrake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    // set max current
    public void setCurrentLimit(boolean enable, double amps) {

        SupplyCurrentLimitConfiguration supplyLimit = new SupplyCurrentLimitConfiguration(
                enable, // enable limit
                amps, // limited to amps
                amps, // when over trigger threshold
                0.20); // for seconds

        talon.configSupplyCurrentLimit(supplyLimit);
    }

    @Override
    public void updateMotorStats() {
        motorEncoder = talon.getSelectedSensorPosition(primaryController);
        motorPos = motorEncoder / countsPerUnit;
        motorPosErr = talon.getClosedLoopError(primaryController) / countsPerUnit;
        motorVel = talon.getSelectedSensorVelocity(primaryController) / speedScale;
    }

    @Override
    // stop motion
    public void stop() {
        talon.set(0);
        motorPwr = 0;
    }

    @Override
    // update state variables from the motor
    public void zeroMotorPos() {
        talon.setSelectedSensorPosition(0, primaryController, timeoutMs);
        motorPos = 0;
        targetPos = 0;
    }

    // set closed loop controller to targetPos (units)
    @Override
    public void snapTo(double pos) {
        talon.set(ControlMode.Position, pos * countsPerUnit);
        this.targetPos = pos;
    }

    // move to targetPos using current cruiseVel and accel constraints
    @Override
    public void moveTo(double pos, double vel, double acc) {
        talon.configMotionCruiseVelocity(vel * speedScale, 0);
        talon.configMotionAcceleration(acc * speedScale, 0);
        talon.set(ControlMode.MotionMagic, pos * countsPerUnit);
        targetPos = pos;
    }

    // start moving using (-1 -> 1)
    public void moveByPower(double fraction) {
        talon.set(fraction);
        motorPwr = fraction;
    }

    // start moving at voltagle (-1 -> 1)
    public void moveByVoltage(double fraction) {
        talon.set(ControlMode.PercentOutput, fraction);
        motorPwr = fraction;
    }
}
