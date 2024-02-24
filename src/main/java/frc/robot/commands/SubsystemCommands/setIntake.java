// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.subsystemConstants;
import frc.robot.subsystems.intakeSubsystem;

public class setIntake extends Command {
  /** Creates a new setIntake. */
  intakeSubsystem sIntake;
  double setpoint;
  boolean useNoteSensor,autonMode;
  int     noteSensorVal;
  public setIntake(intakeSubsystem sIntake, double setpoint,boolean useNoteSensor,boolean autonMode) {
    this.sIntake = sIntake;
    this.setpoint = setpoint;
    this.useNoteSensor = useNoteSensor;
    this.autonMode = autonMode;
    addRequirements(sIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sIntake.setAutonMode(autonMode);
       if (noteSensorVal<Constants.subsystemConstants.kNoteDetectedValueUL && noteSensorVal>subsystemConstants.kNoteDetectedValueLL && noteSensorVal>subsystemConstants.kNoteDetectedValueLL){
     sIntake.setSpeed(setpoint);

    }else if(!useNoteSensor){
           sIntake.setSpeed(setpoint);
    }else {
      sIntake.setSpeed(setpoint);
      System.out.println("NOTE SENSOR ON AT START OF INTAKE");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
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
    if(autonMode){
      return true;
    }
    if (useNoteSensor){
       if (sIntake.getNoteSensorVal()<Constants.subsystemConstants.kNoteDetectedValueUL){
        System.out.println("Stopped Due to Note Sensor");
        return true;
       }else return false;
    }else return false;
  }
}
