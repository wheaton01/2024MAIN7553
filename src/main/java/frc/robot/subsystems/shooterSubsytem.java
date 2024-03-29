// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelight;
import frc.robot.Constants.subsystemConstants;

public class shooterSubsytem extends SubsystemBase {
  /** Creates a new shooterSubsytem. */
  CANSparkMax tShooter,bShooter ;
  RelativeEncoder ebShooter,etShooter;
  public double kP, kI, kD, kIz, kFF, kPl,kIl,kDl, kMaxOutput, kMinOutput, maxRPM;
  public double currentSetpoint,shooterTolerance;
  public double currentSetpointBot,currentSetpointTop;
  int topMotor, botMotor;
  PIDController shooterPID;
  SparkPIDController tShooterPID,bShooterPID;
  limelight armLimelight;
  public shooterSubsytem(int topMotor, int botMotor) {
    
    this.topMotor = topMotor;
    this.botMotor = botMotor;
    tShooter = new CANSparkMax(topMotor, CANSparkLowLevel.MotorType.kBrushless);
    bShooter = new CANSparkMax(botMotor, CANSparkLowLevel.MotorType.kBrushless);
    bShooter.setVoltage(12);
    tShooter.setVoltage(12);
    bShooter.setInverted(false);
    tShooter.setInverted(false);//TODO: MAY NEED CHANGES WITH PHYSICAL ROBOT
    etShooter = tShooter.getEncoder();
    ebShooter = bShooter.getEncoder();

    
    // tShooterPID = tShooter.getPIDController();
    kP = 0.00084;//TODO: TUNE PID
    kI = 0.00005000;
    kD = 0.0001;
    kIz = 0;
    kFF = 0.000; // .000015

    kMaxOutput = 1;
    kMinOutput = 0.0;
    kPl = 0.001884;//TODO: TUNE PID
    kIl = 0.000000050;
    kDl = 0.000001;
    

    tShooterPID = tShooter.getPIDController();
    tShooterPID.setP(kP);
    tShooterPID.setI(kI);
    tShooterPID.setD(kD);
    tShooterPID.setIZone(kIz);
    tShooterPID.setFF(kFF);
    tShooterPID.setOutputRange(kMinOutput, kMaxOutput);     
    bShooterPID = bShooter.getPIDController();

    bShooterPID.setP(kP);
    bShooterPID.setI(kI);
    bShooterPID.setD(kD);
    bShooterPID.setIZone(kIz);
    bShooterPID.setFF(kFF);
    bShooterPID.setOutputRange(kMinOutput, kMaxOutput);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //bShooterPID.setReference(currentSetpoint, CANSparkMax.ControlType.kVelocity);  
    //tShooterPID.setReference(currentSetpoint, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("TOP SPEED", getTopVelocity());
    SmartDashboard.putNumber("BOT SPEED", getBotVelocity());
    SmartDashboard.putNumber("Current Setpoint",currentSetpoint);
    
    tShooter.set(currentSetpointTop);
    bShooter.set(currentSetpointBot);
  }
  public void diffentialSpeedSet(double topSpeed, double botSpeed){
    currentSetpointTop = topSpeed;
    currentSetpointBot = botSpeed;
  }
  
  public void setSpeed(double setpoint){
    //bShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    //tShooterPID.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    currentSetpointTop = setpoint;
    currentSetpointBot = setpoint;
  }


  public double getTopVelocity(){
    return etShooter.getVelocity();
  }
  public double getBotVelocity(){
    return ebShooter.getVelocity();
  }
  public boolean inTolerance(){
    if (subsystemConstants.kShooterPIDTolerance>Math.abs(currentSetpoint - etShooter.getVelocity())){
      return true;
    }   
    if (subsystemConstants.kShooterPIDTolerance<Math.abs(currentSetpoint - etShooter.getVelocity())){
      return false;
    }else{ return false;}
  }
  public void setZero(){
    bShooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    tShooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    currentSetpoint=0;
  }


}
