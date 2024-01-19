// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

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

  public static final double ROBOT_MASS = (30) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double kTrackWidth = 0.5588;
  public static final double kWheelBase = 0.5588;
  public static final double kPhysicalMaxSpeedMetersPerSecond = 5;  
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);
    
    public static final double kMaxAngularSpeedRadiansPerSecond = //
    10 / 10; //Originally should be calculated using max angluar speed physically *2*PI/10 IDK where they got these numbers from but maybe one day ill know!
public static final double kMaxAccelerationMetersPerSecondSquared = 3;
public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController =  5.;
    public static final double kPYController =  5.;
    public static final double kPThetaController = .5;
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
    new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      5, 
      0.395224, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds


    public static final double FROffset = 319.6;
    public static final double FLOffset = 346.5;
    public static final double RLOffset = 302.4;
    public static final double RROffset = 305.2;
  }

  public static class OperatorConstants
  {
    public static final int kOperatorPort = 1;
    public static final int kDriverPort = 0;
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double RIGHT_Y_DEADBAND = 0.01;
    public static final double TURN_CONSTANT = .750;
  }
  public static class driveterainConstants{
    //these really only are for reference. drivetrain offsets are set in deploy/swerve/neo/modules respective json files
    public static final double FROffset = 319.6;
    public static final double FLOffset = 346.5;
    public static final double RLOffset = 302.4;
    public static final double RROffset = 305.2;
  }
  public static final class subsystemConstants{
    


    public static final double kShooterPIDTolerance = 30;
    public static final double kArmShootingPos = 20;
    public static final double kShootignSpeed= 3000;
    public static final double kIntakeSpeed = .8;
    public static final double kIntakeFeedSpeed = .5;
    public static final double kWaitForSpool = .1;
    public static final double kWaitForShoot = 2;
    



  }
  public static final class Ports{
    //TODO: GET CORRECT MOTOR IDS
    public static final int kArmMotorID            = 15;
    public static final int kIntakeMotorID         = 16;
    public static final int kTopShooterMotorID     = 17;
    public static final int kBotShooterMotorID     = 19;
    public static final int kWinchMotorID          = 18;
    public static final int kArmEncoderID1         = 0;
    public static final int kArmEncoderID2         = 1;
    public static final int kNoteSensorID          = 2;
    public static final int kLowerLimitID          = 3;
  
  }
}
