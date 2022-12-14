// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc2016.commands;

import org.usfirst.frc2016.Robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimberAuto extends CommandGroup {
  /** Add your docs here. */
  public ClimberAuto() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.


    /*
      do together : flop out
                  extend out
      flop in
      extend in
    */
    
    double extendSpeed = 0.5;
    double extendTime = 2.25;

    double liftSpeed = 0.3;
    double liftTime = 5.5;

    double flopSpeed = 1;
    double flopTime = 1.0;

    //CommandGroup par = new CommandGroup();
    //par.addParallel(new ClimberFlop(flopSpeed), flopTime);
    //par.addParallel(new ClimberExtend(liftSpeed, liftSpeed, extendTime));
    
    
    //addSequential(par);

    addSequential(new ClimberFlop(flopSpeed), flopTime);
    addSequential(new ClimberExtend(-extendSpeed, -extendSpeed), extendTime);

    addSequential(new ClimberFlop(-flopSpeed), flopTime);
    addSequential(new ClimberExtend(liftSpeed, liftSpeed), liftTime);
 
  }
}
