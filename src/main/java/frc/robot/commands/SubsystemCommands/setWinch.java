// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armSubsystem;

public class setWinch extends Command {
  /** Creates a new setWinch. */
  boolean winchReverse,winchForeward;
  armSubsystem sArm;
  public setWinch(armSubsystem sArm,boolean winchReverse,boolean winchForeward) {
    this.winchForeward = winchForeward;
    this.winchReverse  = winchReverse;
    this.sArm          = sArm;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (winchForeward){sArm.setWinch(1.0);
    }
    if (winchReverse){sArm.setWinch(-1);
    }else {sArm.setWinch(0);}
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
