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

public class ColorSensor_Subsystem extends SubsystemBase {
  /**
   * Creates a new ColorSensor_Subystem.
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
    colorWheelMotor = new Victor(9);

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

  public String getColor() {
    ColorMatchResult match = colorMatcher.matchClosestColor(colorSensor.getColor());

    String color;
    if (match.color == red) {
      color = "red";
    } else if (match.color == green) {
      color = "green";
    } else if (match.color == blue) {
      color = "blue";
    } else if (match.color == yellow) {
      color = "yellow";
    } else {
      color = "unkown";
    }

    return color;
  }

  public void spinMotor(double speed) {
    colorWheelMotor.set(speed);
    System.out.println(colorWheelEncoder.getDistance());
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
