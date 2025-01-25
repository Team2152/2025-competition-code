// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  /**
   * The Constants for the Drivetrain.
   */
  public static final class DriveConstants
  {
    /**
     * The unique ID for the Left Front (motor type here) Motor. 
     * This should be used as an Enum. 
     * <p>This is not the internal Motor ID, it is simply a easy-to-read & unique value for each motor.
     */
    public static final int LEFT_FRONT = 1;
    /**
     * The unique ID for the Left Back (motor type here) Motor. 
     * This should be used as an Enum. 
     * <p>This is not the internal Motor ID, it is simply a easy-to-read & unique value for each motor.
     */
    public static final int LEFT_BACK = 2;
    /**
     * The unique ID for the Right Front (motor type here) Motor. 
     * This should be used as an Enum. 
     * <p>This is not the internal Motor ID, it is simply a easy-to-read & unique value for each motor.
     */
    public static final int RIGHT_FRONT = 3;
    /**
     * The unique ID for the Right Back (motor type here) Motor. 
     * This should be used as an Enum. 
     * <p>This is not the internal Motor ID, it is simply a easy-to-read & unique value for each motor.
     */
    public static final int RIGHT_BACK = 4;
  }
}
