// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class manualSwerve extends Command {


  SwerveSubsystem swerve;
  DoubleSupplier vX, vY, heading;
  ChassisSpeeds desiredSpeeds;
  SwerveDriveOdometry startingPos;
  Pose2d desiredPos;

  /** Creates a new swerveCrawl. */
  public manualSwerve(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
  DoubleSupplier heading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPos.getPoseMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
    new Rotation2d(heading.getAsDouble() * Math.PI));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
