// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public double setpoint,currentPose;
  int motorID,encoderPort1,encoderPort2,winchMotorID,lowerLimitPort;
  
  //Object Creation
  Encoder angEncoder;
  SparkPIDController mArmPID;
  CANSparkMax armMotor,winchMotor;
  DigitalInput dLowerLimit;
  RelativeEncoder armPosEncoder;

  public armSubsystem(int motorID,int winchMotorID,int lowerLimitPort, int encoderPort1, int encoderPort2) {
    this.motorID = motorID;
    this.encoderPort1 = encoderPort1;
    this.encoderPort2 = encoderPort2;
    this.lowerLimitPort = lowerLimitPort;

    dLowerLimit = new DigitalInput(lowerLimitPort);
    armPosEncoder = armMotor.getEncoder();
    //angEncoder = new Encoder(encoderPort1,encoderPort2);
    armMotor = new CANSparkMax(motorID,CANSparkLowLevel.MotorType.kBrushless);
    armMotor = new CANSparkMax(motorID, CANSparkLowLevel.MotorType.kBrushless);
    winchMotor = new CANSparkMax(winchMotorID, CANSparkLowLevel.MotorType.kBrushless);
    
    //armMotor.getAlternateEncoder(2048);//should be okay
     mArmPID = armMotor.getPIDController();
    //TODO: ZERO ENCODER
    resetEncoder();
  
    // angEncoder.setDistancePerPulse(360.0/2048.0);//will return 360 units for every 2048 pulses which should be the hex shaft encoders value
    kP = 0.0004;//TODO: TUNE PID HERE
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00017; // .000015
    kMaxOutput = 1;
    kMinOutput = -1;

    mArmPID.setP(kP);
    mArmPID.setI(kI);
    mArmPID.setD(kD);
    mArmPID.setIZone(kIz);
    mArmPID.setFF(kFF);
    mArmPID.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    //just polls the encoder angle maybe for command stuff idk yet!
    currentPose = armPosEncoder.getPosition();
    SmartDashboard.putNumber("CURRENT ARM POSE",currentPose);
    // This method will be called once per scheduler run
  }
  public boolean checkLowerLimit(){
    return dLowerLimit.get();
  }
  //sets the arm to a certain pose
  double position;
  public void setPose(double desiredPosition){
    position = desiredPosition * 5.688888;
    //with the encoder reading 2048 ticks per full rotation, it is 5.68... encoder ticks per degree

    mArmPID.setReference(position, CANSparkMax.ControlType.kPosition);
    SmartDashboard.putNumber("CURRENT ARM SETPOINT",position);

  }
  //sets winch speed, plan on using operator stick values for this
  public void setWinch(double setpoint){
    winchMotor.set(setpoint);
  }
  //gets current angle 
  public double getAngle(){
    return angEncoder.getDistance();
  }
  //resets encoder
  public void resetEncoder(){
    angEncoder.reset();

  }
}