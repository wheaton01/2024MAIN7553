// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;

public class setAnalogIntake extends Command {
  /** Creates a new setIntake. */
  intakeSubsystem sIntake;
  DoubleSupplier sendForeward, sendReverse;
  
  boolean useNoteSensor,noteSensorVal;
  public setAnalogIntake(intakeSubsystem sIntake, DoubleSupplier sendForeward,DoubleSupplier sendReverse,boolean useNoteSensor) {
    this.sIntake = sIntake;
    this.sendForeward = sendForeward;
    this.sendReverse = sendReverse;
    this.useNoteSensor = useNoteSensor;



    addRequirements(sIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        sIntake.setSpeed(sendForeward.getAsDouble()-sendReverse.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(useNoteSensor){
      sIntake.setSpeed(0);
      System.out.println("NOTE SENSOR ON, INTAKE OFF");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (useNoteSensor){
      return sIntake.getNoteSensor();
    }else return false;
  }
}
