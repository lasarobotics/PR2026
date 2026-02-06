// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final int kDriverControllerPort = 0;
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
    public static final LinearVelocity MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final LinearAcceleration MAX_ACCELERATION =
        MetersPerSecondPerSecond.of(3); // TODO measure
    public static final AngularVelocity MAX_ANGULAR_RATE =
        RotationsPerSecond.of(0.75); // TODO measure
    public static final AngularAcceleration MAX_ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1); // TODO
    // measure

    public static final double DEADBAND_SCALAR = 0.1;
    public static final double SLOW_SPEED_SCALAR = 0.3;
    public static final double FAST_SPEED_SCALAR = 0.6; // TODO

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

    public static final double AUTO_ALIGN_TOLERANCE = Meters.of(0.085).in(Meters);
    public static final double AUTO_ALIGN_LR_TOLERANCE = Centimeter.of(0.25).in(Meters);
    public static final double AUTO_ALIGN_TOLERANCE_TURN =
        Radians.of(0.075).plus(Degrees.of(7.5)).in(Radians);

    public static final double TURN_P = 0.08;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;
 }
 public static class HubConstants{
  public static final Translation2d BLUE_HUB_POS = new Translation2d(4.7,3);
  public static final Translation2d RED_HUB_POS = new Translation2d(4.7,3);
 }
}
