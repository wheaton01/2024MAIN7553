// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class winchSubsystem extends SubsystemBase {
  /** Creates a new winchSubsystem. */
  double winchMotorID;
   CANSparkMax winchMotor;
  
  public winchSubsystem(int winchMotorID) {
        winchMotor = new CANSparkMax(winchMotorID, CANSparkLowLevel.MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWinch(double setpoint){
    winchMotor.set(setpoint);
  }
}
