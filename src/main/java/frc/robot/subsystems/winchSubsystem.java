// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class winchSubsystem extends SubsystemBase {
  /** Creates a new winchSubsystem. */
  double winchMotorID;
   CANSparkMax winchMotor;
   RelativeEncoder winchEncoder;
  double maxTravel;
  public winchSubsystem(int winchMotorID) {
        winchMotor = new CANSparkMax(winchMotorID, CANSparkLowLevel.MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();
    winchEncoder.setPosition(0);
    maxTravel = 225.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Winch Position",winchEncoder.getPosition());
    // This method will be called once per scheduler run
  }


  public void setWinch(double setpoint){
    if (maxTravel>Math.abs(winchEncoder.getPosition())){
    System.out.println("new Winch Setpoint"+setpoint);
    winchMotor.set(setpoint);
    }else if(maxTravel<Math.abs(winchEncoder.getPosition())){
      System.out.println("MAX TRAVEL WINCH");
      winchMotor.set(0);
    }
  }
}
