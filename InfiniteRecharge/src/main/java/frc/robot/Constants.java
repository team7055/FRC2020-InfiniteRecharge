/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Add motor ports
    public final class Motors {
        public static final int MOTOR_FRONT_RIGHT = 0;
        public static final int MOTOR_FRONT_LEFT = 1;
        public static final int MOTOR_REAR_RIGHT = 2;
        public static final int MOTOR_REAR_LEFT = 3;
        public static final int MOTOR_SHOOTER_1 = 5;
        public static final int MOTOR_SHOOTER_2 = 6;
    }

    // Ports for various joystick axes and buttons
    public final class Controller {
        public static final int JOYSTICK_LEFT_X = 0;
        public static final int JOYSTICK_LEFT_Y = 1;
        public static final int JOYSTICK_RIGHT_X = 4;
        public static final int JOYSTICK_RIGHT_Y = 5;
    }
}
