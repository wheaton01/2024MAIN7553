// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSubsystem extends SubsystemBase {
  /** Creates a new intakeSubsystem. */
  int motorID,noteSensorPort;
  TalonFX intakeM;
  CANSparkMax intakeMC;
  //DigitalInput noteSensor;
  AnalogInput noteSensor;
  public intakeSubsystem(int motorID, int noteSensorPort) {
    this.motorID = motorID;
    this.noteSensorPort = noteSensorPort;
    noteSensor = new AnalogInput(noteSensorPort);
    intakeM = new TalonFX(motorID);
    //intakeMC = new CANSparkMax(motorID, MotorType.kBrushed);//TODO: NOT SURE IF WE ARE USING SPARKMAX OR TALON
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run

  }
  public void setSpeed(double setpoint){
    intakeM.set(ControlMode.PercentOutput, setpoint);
  }
  public int getNoteSensorVal(){
    return noteSensor.getValue();
  }
}
