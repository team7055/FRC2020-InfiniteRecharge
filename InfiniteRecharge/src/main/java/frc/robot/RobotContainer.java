/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.ColorSensor_Command;
import frc.robot.commands.Drive_Command;
import frc.robot.commands.PositionControlReset_Command;
import frc.robot.commands.PositionControl_Command;
import frc.robot.subsystems.ColorSensor_Subsystem;
import frc.robot.commands.Shooter_Command;
import frc.robot.subsystems.Drivetrain_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;
import frc.robot.Constants.Colors.Colour;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.PIDVals.*;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain_Subsystem drivetrain = new Drivetrain_Subsystem();

  private final ColorSensor_Subsystem colorSensor = new ColorSensor_Subsystem();

  private final PositionControl_Command positionControl = new PositionControl_Command(colorSensor, SETPOINT);

  private final PositionControlReset_Command positionControlReset = new PositionControlReset_Command(colorSensor);
  
  private final Shooter_Subsystem shooter = new Shooter_Subsystem();
  // Testing!!
  // Seeing if it is okay to create a new command for the button
  private final ColorSensor_Command colorSensorCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(Colour target) {
    colorSensorCommand = new ColorSensor_Command(colorSensor, colorSensor.calcActualTarget(target));

    CameraServer.getInstance().startAutomaticCapture();

    // Configure the button bindings
    // Pass in the target color to bindings so we can use it in our command
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Joystick driveStick = new Joystick(0);

    drivetrain.setDefaultCommand(new Drive_Command(
      drivetrain,
      driveStick
    ));

    JoystickButton rotationControlButton = new JoystickButton(driveStick, Constants.Controller.JOYSTICK_A_BUTTON);
    
    rotationControlButton.whileHeld(positionControl);
    rotationControlButton.whenInactive(positionControlReset);

    JoystickButton positionControlButton = new JoystickButton(driveStick, 3);

    positionControlButton.whileHeld(colorSensorCommand);
    
    shooter.setDefaultCommand(new Shooter_Command(
      shooter, 
      driveStick
    ));
  }

  // Have a public getter so we can use this command in teleop periodic
  public Command getDrivetrainDefault() {
    return drivetrain.getDefaultCommand();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String trajectoryJSON = "paths/YourPath.wpilib.json";
    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    //MecanumControllerCommand controller = new MecanumControllerCommand();

    // RamseteCommand ramsete = new RamseteCommand(
    //   trajectory, 
    //   drivetrain::getPose,
    //   new RamseteController(2.0, 0.7), // change constants later
    //   new SimpleMotorFeedforward(0.0003, 12.0 / 5130.0),
    //   drivetrain.getKinematics(),
    //   drivetrain::getWheelSpeeds,
    //   new PIDController(0.0, 0.0, 0.0), // add P term later
    //   new PIDController(0.0, 0.0, 0.0),
    //   );
    /*
    Arguments:
    trajectory
    pose -- odometry
    controller
    feedforward
    kinematics
    left/right controller
    output volts
    requirements

    WPI Example:
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );
    */
    
    return null; // finish
  }
}
