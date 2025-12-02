// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static class MiscConstants {
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  }

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    public static final double WHEEL_LOCK_TIME = 10.0; // Hold time on motor brakes when disabled in seconds
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound //TODO set with correct value
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double MAX_SPEED  = Units.feetToMeters(14.5); // Maximum speed of the robot in meters per second, used to limit acceleration.
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class ShooterConstants{
    public static final int SHOOTER_ID= 20; //placeholder
    public static final boolean SHOOTER_INVERTED = false; //maybe change
    public static final int SHOOTER_STALL_LIMIT = 30; //placeholder
    public static final int SHOOTER_FREE_LIMIT = 35; //placeholder
    public static final double SHOOTER_MAX_VELOCITY_RPM = 5676;
    public static final double SHOOTER_VELOCITY_HIGH = 1.0; // placeholder
    public static final double SHOOTER_VELOCITY_LOW = 0.4; // placeholder
  }

  public static class MidstageConstants {
    public static int MIDSTAGE_ID = 30; //placeholder
    public static int BEAMBREAK_ID = 0;
    public static double POSITION_CONVERSION_FACTOR = 1.0;
    public static double MIDSTAGE_SPEED = 0.5; // placeholder
    public static boolean MIDSTAGE_INVERTED = false; // change later
    public static int MIDSTAGE_STALL_LIMIT = 30; // placeholder
    public static int MIDSTAGE_FREE_LIMIT = 35; // placeholder
  }

  public static class LEDConstants {
    public static int kPwmPort = 9; // PWM port for the LED strip
    public static int kLedLength = 300; // Number of LEDs in the strip
    public static double kBrightness = 1; // 100% brightness
  }
}
