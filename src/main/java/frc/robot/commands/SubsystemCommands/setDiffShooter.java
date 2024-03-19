// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.subsystemConstants;
import frc.robot.subsystems.shooterSubsytem;

public class setDiffShooter extends Command {
  /** Creates a new setShooter. */
  shooterSubsytem sShooter;
  
  double botSetpoint, topSetpoint, botVelocity, topVelocity;
  boolean turnOff;
  public setDiffShooter(shooterSubsytem sShooter, double topSetpoint,double botSetpoint, boolean turnOff, boolean useLimelight) {
    this.sShooter = sShooter;
    this.botSetpoint = botSetpoint;
    this.topSetpoint = topSetpoint;

    this.turnOff  = turnOff;


    addRequirements(sShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sShooter.diffentialSpeedSet(topSetpoint,botSetpoint);
    System.out.println("differntialSetpoint in use :");
    System.out.println(" top Setpoint "+topSetpoint+" BotSetpoint "+botSetpoint);

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
    return true;
  }
}
