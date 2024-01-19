// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class zeroGyroCommand extends Command {
  /** Creates a new zeroGyroCommand. */
  SwerveSubsystem swerve;
  public zeroGyroCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.zeroGyro();
    System.out.println("!!GYRO HAS BEEN ZEROED!!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
