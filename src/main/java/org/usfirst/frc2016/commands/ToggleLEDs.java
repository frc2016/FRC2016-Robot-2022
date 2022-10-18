// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.usfirst.frc2016.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class ToggleLEDs extends InstantCommand {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue = 0;

  /** Add your docs here. */
  public ToggleLEDs() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    if (m_rainbowFirstPixelHue == 0) {
      allianceColor();
    } else {
      off();
    }
    
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }

  private void allianceColor() {
    // For every pixel
    switch (DriverStation.getAlliance()) {
      case Blue:
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setLED(i, Color.kBlue);
        }
        m_rainbowFirstPixelHue = 1;
        break;

      case Red:
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setLED(i, Color.kRed);
        }
        m_rainbowFirstPixelHue = 1;
        break;

      default:
        off();
        m_rainbowFirstPixelHue = 0;
        break;
    }
  }

  private void off() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, Color.kBlack);
    }
    m_rainbowFirstPixelHue = 0;
  }
}
