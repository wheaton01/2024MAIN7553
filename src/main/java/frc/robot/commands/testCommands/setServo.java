// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.testCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.testingSubsystems.servoSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setServo extends InstantCommand {
servoSubsystem servoSubsystem;
double pose;
  public setServo(servoSubsystem servoSub, double pose) {
    this.servoSubsystem = servoSub;
    this.pose = pose;

    addRequirements(servoSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    servoSubsystem.setServoPos(pose);
  }
}
