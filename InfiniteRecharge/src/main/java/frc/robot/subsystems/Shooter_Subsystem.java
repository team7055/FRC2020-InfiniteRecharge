/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

public class Shooter_Subsystem extends SubsystemBase {
  /**
   * Creates a new shooter.
   */
  private Spark motor1, motor2;

  public Shooter_Subsystem() {
    motor1 = new Spark(Motors.MOTOR_SHOOTER_LEFT);
    motor2 = new Spark(Motors.MOTOR_SHOOTER_RIGHT);
  }

  public void spinShooter() {
    motor1.set(1.0);
    motor2.set(-1.0);
  }

  public void stopShooter() {
    motor1.set(0.0);
    motor2.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
