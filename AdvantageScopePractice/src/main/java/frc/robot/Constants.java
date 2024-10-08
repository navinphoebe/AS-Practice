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

public static final double GO_TO_X_IS_FINISHED_ACCEPTABLE_DISTANCE_MARGIN = .5;
public static final int ACCEPTABLE_ANGLE_DIFFERENCE = 5;
public static final double MAX_XRP_VELOCITY = .6;
public static final double GOTOX_RADIANS_SPEED = 3.0;
public static final double XRP_TRACK_WIDTH = 6.0;
}
