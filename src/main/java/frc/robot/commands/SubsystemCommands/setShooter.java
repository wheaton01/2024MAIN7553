// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.subsystemConstants;
import frc.robot.subsystems.shooterSubsytem;

public class setShooter extends Command {
  /** Creates a new setShooter. */
  shooterSubsytem sShooter;
  double setpoint, botVelocity, topVelocity;
  boolean turnOff;
  public setShooter(shooterSubsytem sShooter, double setpoint, boolean turnOff, boolean useLimelight) {
    this.sShooter = sShooter;
    this.setpoint = setpoint;
    this.turnOff  = turnOff;


    addRequirements(sShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sShooter.setSpeed(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (turnOff){
      sShooter.setSpeed(subsystemConstants.kIdleSpeed);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(setpoint-Math.abs(sShooter.getTopVelocity()))>subsystemConstants.kShooterPIDTolerance 
    && Math.abs(setpoint-Math.abs(sShooter.getBotVelocity()))<subsystemConstants.kShooterPIDTolerance){
      System.out.println("Shooter At Speed");
      return true;
    }else return false;
  }
}
