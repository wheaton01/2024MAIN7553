// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.winchSubsystem;

public class setWinch extends Command {
  /** Creates a new setWinch. */
  double setpoint;
  winchSubsystem sWinch;
  public setWinch(winchSubsystem sWinch, double setpoint) {

    this.sWinch= sWinch;
    this.setpoint=setpoint;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sWinch);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sWinch.setWinch(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
