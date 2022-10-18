/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

//package edu.wpi.first.wpilibj.templates;
package org.usfirst.frc2016;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;

/**
 *
 * @author Montagna
 */
// This class provides the system defaults
// These should be updated after tuning the cRio NV RAM incase we need to change
// the cRio
public class Defaults {

    // Drive train
    public static final double DRIVETRAIN_VOLTAGE_LIMIT_DEFAULT = .95;
    public static final double DRIVETRAIN_RAMP_INCREMENT = .2;
    public static final double DRIVETRAIN_P = .15;
    public static final double DRIVETRAIN_I = 0;
    public static final double DRIVETRAIN_D = 0;
    public static final double DRIVETRAIN_F = .6;
    public static final double DRIVETRAIN_MAX = .5;
    public static final int DRIVE_CRUISEVELOCITY = 410;
    public static final int DRIVE_ACCELERATION = 205;
    public static final boolean DRIVE_JOYSQUARE = true;

    // Arm presets
    public static final double ARMPOSITIONTRENCH = 80;
    public static final double ARMPOSITIONMATCHSTART = 135;
    public static final double ARMPOSITIONHANG = 180;
    public static final double ARMPOSITIONCOLORWHEEL = 120;

    public static final int REVERSESOFTLIMIT = 78;
    public static final int FORWARDSOFTLIMIT = 185;
    public static final double ARM_P = 1;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final double ARM_F = 0.1;
    // degrees per second
    public static final int ARMCRUISEVELOCITY = 15;
    public static final int ARMCRUISEACCELERATION = 30;
    // degrees per second

    // TeleArm presets
    public static final double TELEARMPOSITIONTRENCH = 0;
    public static final double TELEARMPOSITIONMATCHSTART = 0;
    public static final double TELEARMPOSITIONHANG = 5;
    public static final double TELEARMPOSITIONGRAB = 25;
    public static final double TELEARMPOSITIONCOLORWHEEL = 10;

    public static final int TELEREVERSESOFTLIMIT = 2210;
    public static final int TELEFORWARDSOFTLIMIT = 3450;
    public static final double TELEARM_P = .15;
    public static final double TELEARM_I = 0;
    public static final double TELEARM_D = .1;
    public static final double TELEARM_F = 0;
    // inches per second below
    public static final int TELEARMCRUISEVELOCITY = 2;
    public static final int TELEARMCRUISEACCELERATION = 1;
    public static final double TELEARMCALSPEED = .3;
    // inches per second above

    // Ball Pickup Speeds
    public static final double BALL_PICKUP_RETRACT_SPEED_DEFAULT = .5;
    public static final double BALL_PICKUP_LOWER_SPEED_DEFAULT = .5;
    public static final double BALL_PICKUP_SPEED_IN = .5;
    public static final double BALL_PICKUP_SPEED_OUT = .7;
    public static final double BALL_PICKUP_SPEED_HOLD = .2;
    public static final int BALL_PICKUP_PEAK_CURRENT = 3;
    public static final int BALL_PICKUP_STALL_CURRENT = 3;
    public static final int BALL_PICKUP_PEAK_CURRENT_DURATION = 300;
    public static final double BALL_PICKUP_P = .01;
    public static final double BALL_PICKUP_I = 0;
    public static final double BALL_PICKUP_D = 0;
    public static final double BALL_PICKUP_F = 0;
    public static final double BALL_PICKUP_I_ZONE = 1;
    public static final double RETRACT_MAX = 1;
    public static final double RETRACT_MIN = 1;
    public static final boolean PICKUPUSENEO = false;
    // Autonomous Time Defaults
    public static final double AUT_SHOOTING_DELAY = .5;

    // Autonomous Other Defaults
    public static final double AUT_ENCODER_DISTANCE_10FT = 100; // Distance in inches
    public static final double AUT_RIGHT_Y = -.8;
    public static final double AUT_LEFT_Y = .8;

    // Gyro
    public static final double GYROP = .095;
    public static final double GYROTURNMAX = .35;

    // Magazine
    public static final double MAGAZINE_FORWARD_SPEED = .5;
    public static final double MAGAZINE_REVERSE_SPEED = .5;

    // Toss
    public static final double TOSS_SPEED = .1;
    public static final double TOSS_P = 3;
    public static final double TOSS_I = 0;
    public static final double TOSS_D = 0;
    public static final double TOSS_F = 1.9;
    public static final boolean TOSS_USE_VELOCITY = true;
}
