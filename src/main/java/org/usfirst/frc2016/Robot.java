// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc2016;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc2016.commands.*;
import org.usfirst.frc2016.subsystems.*;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc2016.Config;
import org.usfirst.frc2016.utils.MotorBase;

import edu.wpi.first.vision.VisionThread;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public enum endgamePreset {
        MatchStart, Trench, ColorWheel, Hang, Climb
    };

    public static OI oi;
    public static Config config;
    public static Gyro gyro;
    public static LCTelemetry telem;
    private static String CONFIG_FILE_NAME = "/c/robot.cfg";
    public static Constants constants;
    public static Boolean showDebugDashboard = true;
    public static Boolean isTelemetryEnbled = false;
    public static AutoPilotRobot autoPilotRobot;
    public static PathPlanner pathPlanner;
    public static Timer gameTimer = new Timer();
    public static Boolean gameTimerArmed = false;

    public static final int IMG_WIDTH = 320;
    public static final int IMG_HEIGHT = 240;
    public static VisionThread visionThread;
    public static double centerX = 0.0;
    public static final Object imgLock = new Object();

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static DriveTrainSRX driveTrainSRX;
    public static BallIntake ballIntake;
    public static BallToss ballToss;
    public static Limelight limelight;
    public static Climber climber;

    public static Alliance allianceColor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Boolean firstBallCollected = false;
    public Boolean secondBallCollected = false;
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        readConfig();
        loadConfig();

        allianceColor = DriverStation.getAlliance();

        telem = new LCTelemetry(); // create telem handle.
        telem.loadConfig(config);
        gyro = new Gyro(false);

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrainSRX = new DriveTrainSRX();
        ballIntake = new BallIntake();
        ballToss = new BallToss();
        limelight = new Limelight();
        climber = new Climber();
        // pathPlanner = new PathPlanner();

        autoPilotRobot = new AutoPilotRobot();
        autoPilotRobot.loadFile();

        // UsbCamera camera = CameraServer.startAutomaticCapture();
        // camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

        
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        // (which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.setDefaultOption("Play Dead", new AutoPilotCommand("", true));

        for (String s : Robot.autoPilotRobot.sequenceNames()) {
            chooser.addOption(s, new AutoPilotCommand(s, true));
        }

        SmartDashboard.putData("Auto mode", chooser);
    }

    /**
     * This function is called when the disabled button is hit. You can use it to
     * reset subsystems before shutting down.
     */
    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {
        updateDashboard();
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        gameTimerStart();
        autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
        if (autonomousCommand != null)
            autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        gameTimerCheckFinish();
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        gameTimerStart();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
            autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        MotorBase.updateAll();
        Scheduler.getInstance().run();
        gameTimerCheckFinish();
        updateDashboard();
    }

    public static void gameTimerStart() {
        gameTimer.reset();
        gameTimer.start();
    }

    public void gameTimerCheckFinish() {
        // show the timer values
        SmartDashboard.putString("Course Time", String.format("%.3f", gameTimer.get()));

        XboxController joy = Robot.oi.getdriveJoy();

        if (joy.getRawButton(2)) {
            // 'B' button pressed, reset time and wait for drive state
            gameTimer.stop();
            gameTimer.reset();
            gameTimer.stop();
            gameTimerArmed = true;
        }

        if (gameTimerArmed) {
            // get the driver joysticks
            double joyRight = joy.getRightY();
            double joyLeft = joy.getLeftY();
            if (Math.abs(joyRight) > .1 || Math.abs(joyLeft) > .1) {
                // driving started, start the time
                gameTimerArmed = false;
                gameTimerStart();
            }
        }

        // some course start/finish crossing the line, ignore the first cross
        if (SmartDashboard.getNumber("FinishLine", 0.0) > 0 && gameTimer.get() > 10) {
            gameTimer.stop();
        }
    }

    public static boolean isOperatorBackActive() {
        return false; // Robot.oi.operatorJoy.getRawButton(7);
    }

    public static void readConfig() {
        config = new Config(CONFIG_FILE_NAME);
    }

    private void loadConfig() {

    }

    private void updateDashboard() {
        MotorBase.displayAll();

        // SmartDashboard.putBoolean("UseNeo", usingBallPickupNeo);
        // SmartDashboard.putBoolean("Robot Calibrated",robotIsCalibrated);
        // SmartDashboard.putNumber("Left
        // Encoder",Robot.driveTrainSRX.getLeftEncoder());
        // SmartDashboard.putNumber("Left Distance ",
        // Robot.driveTrainSRX.getLeftDistance());
        // SmartDashboard.putNumber("Left Encoder Vel",
        // Robot.driveTrainSRX.getLeftEncoderVelocity());

        // SmartDashboard.putNumber("Right Encoder",
        // Robot.driveTrainSRX.getRightEncoder());
        // SmartDashboard.putNumber("Right Distance ",
        // Robot.driveTrainSRX.getRightDistance());
        // SmartDashboard.putNumber("Right Encoder Vel",
        // Robot.driveTrainSRX.getRightEncoderVelocity());
        // --SmartDashboard.putNumber("Encoder Rate", driveTrainSRX.getAverageRate());

        // --SmartDashboard.putString("PIMode", PIMode);
        // --SmartDashboard.putBoolean("Cube Detector", !RobotMap.cubeDetector.get());

    }

}
