/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Colors;
import static frc.robot.Constants.Encoders.*;
import frc.robot.Constants.Colors.Colour;

public class ColorSensor_Subsystem extends SubsystemBase {
  /**
   * Creates a new Colorensor_Subystem.
   */

  private I2C.Port i2cPort;
  private final ColorSensorV3 colorSensor;
  private final ColorMatch colorMatcher;
  private final Color red, green, blue, yellow;

  private Encoder colorWheelEncoder;
  private Victor colorWheelMotor;

  public ColorSensor_Subsystem() {
    // Set the encoder for the color wheel's motor
    colorWheelEncoder = new Encoder(MOTOR_FRONT_RIGHT_ENCODER_A, MOTOR_FRONT_RIGHT_ENCODER_B);

    // Set the encoder's distance per pulse
    // See explanation in drivetrain for in depth reasoning for variables/parameters
    colorWheelEncoder.setDistancePerPulse(SMALL_MOTOR_DIST_PER_PULSE);

    // Set the motor that will be used to move the color wheel
    colorWheelMotor = new Victor(4);

    // Set i2c port to the roborio port
    i2cPort = I2C.Port.kOnboard;

    // Initialize sensor on the i2c port
    colorSensor = new ColorSensorV3(i2cPort);

    // Initialize colors (based on ones predetermined) and the color matcher
    // RGB values can be found in Constants.java
    red = ColorMatch.makeColor(Colors.Red.RED, Colors.Red.GREEN, Colors.Red.BLUE);
    green = ColorMatch.makeColor(Colors.Green.RED, Colors.Green.GREEN, Colors.Green.BLUE);
    blue = ColorMatch.makeColor(Colors.Blue.RED, Colors.Blue.GREEN, Colors.Blue.BLUE);
    yellow = ColorMatch.makeColor(Colors.Yellow.RED, Colors.Yellow.GREEN, Colors.Yellow.BLUE);

    // Initialize the color matcher, and add our targets to it
    colorMatcher = new ColorMatch();

    colorMatcher.addColorMatch(red);
    colorMatcher.addColorMatch(green);
    colorMatcher.addColorMatch(blue);
    colorMatcher.addColorMatch(yellow);
  }

  // This method returns the color sensors current readed color
  // Returns the color as type Colour (can be found in Constants)
  public Colour getColor() {

    // This variable stores the closest match to the colorSensor's getColor() of the
    // ones stored in the matcher (added above)
    ColorMatchResult match = colorMatcher.matchClosestColor(colorSensor.getColor());

    Colour color;

    // These if statements check which closest color the matcher determined and 
    // assigns it to our color variable
    // Unfortunately, we can't use a switch statement because, reasons
    if (match.color == red) {
      color = Colour.Red;
    } else if (match.color == green) {
      color = Colour.Green;
    } else if (match.color == blue) {
      color = Colour.Blue;
    } else if (match.color == yellow) {
      color = Colour.Yellow;
    } else {
      color = Colour.Unknown;
    }

    // Return the color that was matched
    return color;
  }

  // takes in the target color and returns the setpoint needed to get to that color
  public double numArcLenghtsNeeded(Colour targetColor, Colour currentColor) {
    return 0.0;
  }

  public void spinMotor(double speed) {
    colorWheelMotor.set(speed);
  }

  public Encoder getEncoder() {
    return colorWheelEncoder;
  }

  public Victor getMotor() {
    return colorWheelMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
