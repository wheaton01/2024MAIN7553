// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armSubsystem;

public class armRecovery extends Command {
  /** Creates a new armRecovery. */
  armSubsystem sArm;
  DoubleSupplier setpoint;
  public armRecovery(armSubsystem sArm, DoubleSupplier setpoint) {
    this.sArm= sArm;
    this.setpoint=setpoint;
    addRequirements(sArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sArm.recoveryMode(setpoint.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sArm.resetRecoveryMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
