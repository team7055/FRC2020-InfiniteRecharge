/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Motors.*;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.Encoders.*;

public class Drivetrain_Subsystem extends SubsystemBase {
  /**
   * Creates a new Drivetrain_Subsystem.
   */
  private Talon frontRight;
  private Talon frontLeft;
  private Talon rearRight;
  private Talon rearLeft;

  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;

  private Encoder frontRightEncoder, frontLeftEncoder, 
  rearRightEncoder, rearLeftEncoder;

  private MecanumDrive drivetrain;

  private AHRS gyro;
  private MecanumDriveOdometry odometry;
  private MecanumDriveKinematics kinematics;
  private Pose2d pose;

  private SimpleMotorFeedforward feedforward;
  
  public Drivetrain_Subsystem() {
    // Initialize motors with ports from Constants.java
    frontRight = new Talon(MOTOR_FRONT_RIGHT);
    frontLeft = new Talon(MOTOR_FRONT_LEFT);
    rearRight = new Talon(MOTOR_REAR_RIGHT);
    rearLeft = new Talon(MOTOR_REAR_LEFT);

    // Initialize speed controller groups for trajectory tank drive
    leftMotors = new SpeedControllerGroup(frontLeft, rearLeft);
    rightMotors = new SpeedControllerGroup(frontRight, rearRight);

    // Initialize the locations of wheels relative to robot center
    // Translation2d frontLeftLocation = new Translation2d(-0.254, -0.254);
    // Translation2d frontRightLocation = new Translation2d(-0.254, 0.254);
    // Translation2d backLeftLocation = new Translation2d(0.254, -0.254);
    // Translation2d backRightLocation = new Translation2d(0.254, 0.254);

    Translation2d frontLeftLocation = new Translation2d(-0.254, 0.254);
    Translation2d frontRightLocation = new Translation2d(0.254, 0.254);
    Translation2d backLeftLocation = new Translation2d(-0.254, -0.254);
    Translation2d backRightLocation = new Translation2d(-0.254, 0.254);

    // create the kinematics class with the locations
    kinematics = new MecanumDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    // initialize gyroscope
    gyro = new AHRS(Port.kMXP);
    gyro.reset();

    // create rotation2d object with the gyroscope heading
    Rotation2d heading = Rotation2d.fromDegrees(-gyro.getAngle());

    // create odometry
    odometry = new MecanumDriveOdometry(kinematics, heading);

    // create the feedforward
    feedforward = new SimpleMotorFeedforward(0.557, 0.0578);

    // Initialize encoders on each drive motor
    frontLeftEncoder = new Encoder(DRIVE_FRONT_LEFT_ENCODER_A, DRIVE_FRONT_LEFT_ENCODER_B);
    frontRightEncoder = new Encoder(DRIVE_FRONT_RIGHT_ENCODER_A, DRIVE_FRONT_RIGHT_ENCODER_B);
    rearLeftEncoder = new Encoder(DRIVE_REAR_LEFT_ENCODER_A, DRIVE_REAR_LEFT_ENCODER_B);
    rearRightEncoder = new Encoder(DRIVE_REAR_RIGHT_ENCODER_A, DRIVE_REAR_RIGHT_ENCODER_B);

    frontLeftEncoder.reset();
    frontRightEncoder.reset();
    rearLeftEncoder.reset();
    rearRightEncoder.reset();

     /*
     First of the all the gearbox doesn't matter if the encoder comes after the gearbox.
     Second 0.5 is the diameter of the shaft and multiplying it by PI is the circumference of the shaft.
     And dividing by 2048 (number of pulses per revolution). 

     (2(PI)R) / numOfPulses
            OR
    (Circumference of Shaft) / (Number Of Pulses per Revolution)

     R is the radius of the shaft
     numOfPulses is the number of pulses per revolution of the encoder.


    The diameter of the shaft is the diameter of the shaft coming from the motor.
    The pulses per rev can be found in the datasheet of the encoder. Which in this case is the REV 
    Through-Bore encoder. 

    */

    frontRightEncoder.setDistancePerPulse(DRIVE_MOTOR_DIST_PER_PULSE_METRIC);
    frontLeftEncoder.setDistancePerPulse(DRIVE_MOTOR_DIST_PER_PULSE_METRIC);
    rearRightEncoder.setDistancePerPulse(DRIVE_MOTOR_DIST_PER_PULSE_METRIC);
    rearLeftEncoder.setDistancePerPulse(DRIVE_MOTOR_DIST_PER_PULSE_METRIC);

    // Initialize drive with motors
    drivetrain = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  }

  public void drive(double x, double y, double z) {
    drivetrain.feedWatchdog();
    drivetrain.driveCartesian(x, -y, z);
    //System.out.println(Math.floor(gyro.getAngle()));
    //System.out.println(x + " " + y + " " + z);
  }

  public void drive(MecanumDriveWheelSpeeds speeds) {
    setSpeeds(speeds);
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    PIDController frontLeftController = new PIDController(0.0647, 0, 0);
    PIDController frontRightController = new PIDController(0.0647, 0, 0);
    PIDController rearLeftController = new PIDController(0.0647, 0, 0);
    PIDController rearRightController = new PIDController(0.0647, 0, 0);

    double flFeedforward = feedforward.calculate(speeds.frontLeftMetersPerSecond);
    double frFeedforward = feedforward.calculate(speeds.frontLeftMetersPerSecond);
    double rlFeedforward = feedforward.calculate(speeds.rearLeftMetersPerSecond);
    double rrFeedforward = feedforward.calculate(speeds.rearRightMetersPerSecond);

    double flOutput = frontLeftController.calculate(frontLeftEncoder.getRate(), speeds.frontLeftMetersPerSecond);
    double frOutput = frontRightController.calculate(frontRightEncoder.getRate(), speeds.frontRightMetersPerSecond);
    double rlOutput = rearLeftController.calculate(rearLeftEncoder.getRate(), speeds.rearLeftMetersPerSecond);
    double rrOutput = rearRightController.calculate(rearRightEncoder.getRate(), speeds.rearRightMetersPerSecond);

    driveVolts(flOutput + flFeedforward, frOutput + frFeedforward, rlOutput + rlFeedforward, rrOutput + rrFeedforward);

    frontLeftController.close();
    frontRightController.close();
    rearLeftController.close();
    rearRightController.close();
  }

  public void driveVolts(double flVolts, double frVolts, double rlVolts, double rrVolts) {
    frontLeft.setVoltage(flVolts);
    frontRight.setVoltage(frVolts);
    rearLeft.setVoltage(rlVolts);
    rearRight.setVoltage(rrVolts);
    drivetrain.feed();
  }
  // Shooter Inside Wheel Distance is 5 and 3 quarters inches

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    // Get my wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getRate(), frontRightEncoder.getRate(),
      rearLeftEncoder.getRate(), rearRightEncoder.getRate());

    return wheelSpeeds;
  }

  public Rotation2d getHeading() {
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d heading = Rotation2d.fromDegrees(-gyro.getAngle());

    return heading;
  }

  public MecanumDriveWheelSpeeds getState() {
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getRate(),
      frontRightEncoder.getRate(),
      rearLeftEncoder.getRate(),
      rearRightEncoder.getRate()
    );
  }

  // @param whichEncoder >= 0 & <= 3
  // used to select which encoder to get
  // 0 -> front left encoder,
  // 1 -> front right encoder,
  // 2 -> rear left encoder,
  // 3 -> rear right encoder
  public Encoder getEncoder(int whichEncoder) {
    switch (whichEncoder) {
      case 0:
        return frontLeftEncoder;
      case 1:
        return frontRightEncoder;
      case 2:
        return rearLeftEncoder;
      case 3:
        return rearRightEncoder;
      default:
        return frontLeftEncoder;
    }
  }

  public Talon getFrontLeftMotor() {
    return frontLeft;
  }

  public Talon getFrontRightMotor() {
    return frontRight;
  }

  public Talon getRearLeftMotor() {
    return rearLeft;
  }

  public Talon getRearRightMotor() {
    return rearRight;
  }

  public Pose2d getPose() {
    return pose;
  }

  public MecanumDriveKinematics getKinematics() {
    return kinematics;
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    // Update the pose
    pose = odometry.update(getHeading(), getState());
  }
}
