// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.armSubsystem;

public class setArm extends Command {
  /** Creates a new setArm. */
  armSubsystem sArm;
  double setpoint,currentPose;
  boolean useLimitSwitch,blimitSwitch,useLimelight;

  public setArm(armSubsystem sArm, double setpoint, boolean useLimitSwitch, boolean useLimelight) {
    this.sArm = sArm;
    this.setpoint = setpoint;
    this.useLimitSwitch = useLimitSwitch;
    this.useLimelight = useLimelight;


    addRequirements(sArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sArm.resetRecoveryMode();
    sArm.setPose(setpoint,useLimelight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(useLimitSwitch){
      return sArm.checkLowerLimit();
    }else if (1>Math.abs(setpoint-sArm.getAngle())){
      return true;
    }else return false;
  }
}
