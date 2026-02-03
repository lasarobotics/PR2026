// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

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
    public static final int kDriverControllerPort = 1;
  }

  public static class ClimbConstants {
    public static final PositionVoltage UP_SET_POINT = new PositionVoltage(0.0); // TODO
    public static final PositionVoltage BACK_SET_POINT = new PositionVoltage(0.0); // TODO
    public static final PositionVoltage FORWARD_SET_POINT = new PositionVoltage(0.0); // TODO
    public static final int CLIMB_MOTOR_ID = 0; // TODO
  }

  public static class FuelManagerConstants {
    public static final int SHOOT_MOTOR_ID = 0; // TODO
    public static final int INTAKE_MOTOR_ID = 0; //TODO
    public static final int MIDDLE_MOTOR_ID = 0; // TODO
    public static double SHOOT_MOTOR_SPEED = 0; // TODO
    public static double INTAKE_MOTOR_SPEED = 0; // TODO
    public static double MIDDLE_MOTOR_INTAKE_SPEED = 0; // TODO
    public static double MIDDLE_MOTOR_SHOOT_SPEED = 0; // TODO
  }

  public static class DriveConstants {
    // MAX LINEAR SPEED
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;

    // MAX LINEAR ACCELERATION
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3);

    // MAX TURN SPEED
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75);

    // MAX TURN ACCELERATION
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1);

    // NOT NEEDED RIGHT NOW (WAS IN HEADHONCHO PH2025)
    // public static final double SLOW_SPEED_SCALAR = 0.3;
    // public static final double FAST_SPEED_SCALAR = 1.0;

    // TrapezoidProfile.Constraints(MAX_VELOCITY_PER_SECOND, MAX_ACCELERATION_PER_SECOND_SQUARED)
    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_RATE.in(RadiansPerSecond),
            MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));
    public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_SPEED.in(MetersPerSecond) * 0.85,
            MAX_ACCELERATION.in(MetersPerSecondPerSecond) * 0.6);

    public static final TrapezoidProfile.Constraints TURN_CONSTRAINTS_SLOW =
    new TrapezoidProfile.Constraints(
        MAX_ANGULAR_RATE.in(RadiansPerSecond),
        MAX_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));
    public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS_SLOW =
        new TrapezoidProfile.Constraints(
            MAX_SPEED.in(MetersPerSecond) * 0.25,
            MAX_ACCELERATION.in(MetersPerSecondPerSecond) * 0.2);

    // TODO TODO TODO
    public static final double AUTO_SHOOT_MAX_DISTANCE = Meters.of(0.0).in(Meters);
    // TODO: ADD MORE

    // TODO find real value
    public static final double s_driveSpeedScaler = .8;

    // TODO TODO TODO REVISE
    public static final double TURN_P = 0.01;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
    // TODO TODO TODO

    public static final Translation2d HUB_TRANSLATION_COORDINATES_BLUE = new Translation2d(0,0);
    public static final Translation2d HUB_TRANSLATION_COORDINATES_RED = new Translation2d(0,0);

    public static final double DEADBAND_SCALAR = .085;

    // TODO FIND REAL POSSIBLE POSITIONS
    public static final Translation2d[] RED_TOWER_COORDINATES = {
      new Translation2d(0,0),
      new Translation2d(0,0),
    };

    public static final Translation2d[] BLUE_TOWER_COORDINATES = {
      new Translation2d(0,0),
    };
  }
}
