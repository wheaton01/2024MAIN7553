// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testingSubsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class servoSubsystem extends SubsystemBase {
  /** Creates a new servoSubsystem. */
  Servo testServo;
  public servoSubsystem() {
  testServo = new Servo(0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setServoPos(double pose){
    testServo.set(pose);

  }
    public void setServoZero(){
    testServo.set(0);

  }
}
