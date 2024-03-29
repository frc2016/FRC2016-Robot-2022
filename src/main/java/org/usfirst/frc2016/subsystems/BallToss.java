package org.usfirst.frc2016.subsystems;

import org.usfirst.frc2016.utils.*;
import org.usfirst.frc2016.commands.*;
import org.usfirst.frc2016.Robot;
import org.usfirst.frc2016.Config;
import org.usfirst.frc2016.Constants;
import org.usfirst.frc2016.Defaults;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallToss extends Subsystem {

    static final int UPPER = 0;
    static final int LOWER = 1;
    static final int GATE = 2;
    static final int MAX_MOTORS = 3;

    private class BallTossMotor {
        MotorBase motor;
        double speedSet;
        double speedActual;
        double speedShown;

        public BallTossMotor(MotorBase motor) {
            this.motor = motor;
            // set scaling for fractional speed control: 0.0 -> 1.0
            motor.setUnits("fraction", motor.getCntsPerRev() * motor.getMaxRevsPerSec());

            motor.setUnitsNative();
            boolean i = true;
            while (i) {
                if (readyToToss) {
                    break;
                }
            }

        }

        void setMotorSpeed(double vel) {
            motor.moveByPower(vel);
            speedSet = vel;
        }

        double getMotorSpeed() {
            //motor.updateMotorStats();
            speedActual = motor.getMotorVel();
            return speedActual;
        }

        void slowDown() {
            setMotorSpeed(Math.max(0.0, speedSet - motor.getMoveVel() * Robot.kDefaultPeriod / stopSeconds));
        }

        void start(boolean fwd) {
            motor.zeroMotorPos();
            setMotorSpeed(fwd ? motor.getMoveVel() : -motor.getMoveVel());
        }

        void stop() {
            motor.stop();
            speedSet = 0;
        }

    };

    private BallTossMotor[] motors = new BallTossMotor[MAX_MOTORS];

    private DigitalInput digitalInputReadyToToss = new DigitalInput( Constants.DigitalInputs.readyToToss);
    boolean readyToToss = false;
    private long nanoTimeStart; // nanoseconds
    private double stopSeconds = 0.250;

    public BallToss() {
        MotorBase upper = new MotorFalcon("Toss", "upper", Constants.DeviceId.tossUpper, 0, false);

        MotorBase lower = new MotorFalcon("Toss", "lower", Constants.DeviceId.tossLower, 0, false);

        MotorBase gate = new MotorNeo("Gate", "", Constants.DeviceId.tossGate, 0, true);
        gate.setDisplayFormats(2, 3, 3, 0);

        motors[UPPER] = new BallTossMotor(upper);
        motors[LOWER] = new BallTossMotor(lower);
        motors[GATE] = new BallTossMotor(gate);
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
        readyToToss = digitalInputReadyToToss.get();
        SmartDashboard.putBoolean("ReadyToToss", readyToToss);
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /*
     * Method for setting the toss speed to a preset For now use % voltage
     */

    public void showSpeeds() {
        boolean show = false;
        for (BallTossMotor m : motors) {
            show |= Math.abs(m.speedShown - m.getMotorSpeed()) > .01;
        }

        // show = true;
        if (show) {
            StringBuilder sb = new StringBuilder();
            sb.append(String.format("%6.3f : toss speed", getElapsedSeconds()));
            for (BallTossMotor m : motors) {
                m.speedShown = m.speedActual;
                sb.append(String.format("  %.2f", m.speedShown));
            }
            System.out.println(sb.toString());
        }
    }

    public void start() {
        nanoTimeStart = System.nanoTime();
        for (BallTossMotor m : motors) {
            m.start(true);
        }
    }

    public void slowDown() {
        for (BallTossMotor m : motors) {
            m.slowDown();
        }
    }

    public void stop() {
        for (BallTossMotor m : motors) {
            m.stop();
        }
    }

    public boolean isStopped() {
        boolean isStopped = true;
        for (BallTossMotor m : motors) {
            isStopped &= m.speedSet <= 0.0;
        }
        return isStopped;
    }

    public void gateTurn(int encoderCount){
        motors[BallToss.GATE].speedSet = 0.1;
        motors[BallToss.GATE].motor.moveTo(encoderCount);
    }

    public void gateStart(boolean fwd) {
        motors[BallToss.GATE].start(fwd);
    }


    private void loadConfig(Config config) {
        // motors[UPPER].speedToss = config.getDouble("Toss_SpeedUpper", Defaults.TOSS_SPEED);
        // motors[LOWER].speedToss = config.getDouble("Toss_SpeedLower", Defaults.TOSS_SPEED);
        // motors[GATE].speedToss = config.getDouble("Toss_SpeedGate", Defaults.TOSS_SPEED);
        // tossP = config.getDouble("TossP", Defaults.TOSS_P);
        // tossI = config.getDouble("TossI", Defaults.TOSS_I);
        // tossD = config.getDouble("TossD", Defaults.TOSS_D);
        // tossF = config.getDouble("TossF", Defaults.TOSS_F);
    }

    public double getElapsedSeconds() {
        return (double) (System.nanoTime() - nanoTimeStart) / 1e9;
    }

}
