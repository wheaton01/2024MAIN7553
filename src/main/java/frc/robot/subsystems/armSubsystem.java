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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelight;

public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  public double kP, kI, kD, kIz, kFF,kPUP,kDUP,kFFUP,kIUP, kMaxOutput, kMinOutput, maxRPM, oldPose,newPose;
  public double setpoint,currentPose, limelightTa,limelightTx,limelightTy;
  public double kPToHome,kIToHome,kDToHome;
  int motorID,encoderPort1,encoderPort2,winchMotorID,lowerLimitPort;
  
  //Object Creation
  Encoder angEncoder;
  CANSparkMax armMotor;
  DigitalInput dLowerLimit;
  limelight       armLimelight;
  PIDController   armPID;
  ProfiledPIDController armPrPID;
  boolean       usingLimelight,bErrorFlag;
  public armSubsystem(int motorID, int lowerLimitPort, int encoderPort1, int encoderPort2) {

    this.motorID = motorID;
    this.encoderPort1 = encoderPort1;
    this.encoderPort2 = encoderPort2;
    this.lowerLimitPort = lowerLimitPort;
    
    armLimelight = new limelight();
    dLowerLimit = new DigitalInput(lowerLimitPort);
    angEncoder = new Encoder(encoderPort1,encoderPort2);
    armMotor = new CANSparkMax(motorID,CANSparkLowLevel.MotorType.kBrushless);
    //armMotor.getAlternateEncoder(2048);//should be okay
    //TODO: ZERO ENCODER
    resetEncoder();
    
    // angEncoder.setDistancePerPulse(360.0/2048.0);//will return 360 units for every 2048 pulses which should be the hex shaft encoders value
    kP = 0.015588;//TODO: TUNE PID HERE
    kI = 0.000385;
    kD = 0.001300;
    armPrPID = new ProfiledPIDController(kP,kI,kD, new TrapezoidProfile.Constraints(350,600)) ;
    armPrPID.setTolerance(1.5);
    // kPUP = 0.00290832;//used for when its going up to prevent unspooling
    // kIUP = .00000005;    
    // kDUP = .000000001;

    // kPToHome = 0.008832;//used for when its going up to prevent unspooling
    // kIToHome = .000001;    
    // kDToHome = .0;    
    kIz = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    armPID = new PIDController(kP, kI,kD);
    
    angEncoder.setDistancePerPulse(360.0/2048.0);
    SmartDashboard.putNumber("ArmKP", kP);
    SmartDashboard.putNumber("ArmKI", kI);
    SmartDashboard.putNumber("ArmKD", kD);

    armPID.setIZone(kIz);
    armPID.setTolerance(.5);
    bErrorFlag = false;
    oldPose= 0;
    newPose= 0;
  }

  @Override
  public void periodic() {
    //just polls the encoder angle maybe for command stuff idk yet!
    currentPose = angEncoder.getDistance();
    newPose = getAngle();
    if(Math.abs(oldPose)-Math.abs(newPose)>10){
      bErrorFlag = true;
    }
    armPrPID.setGoal(setpoint);
    if (bErrorFlag) {
      armMotor.set(0);
      System.out.println("ARM IS IN ERROR STATE");
    }
    if(!bErrorFlag){
    if (!usingLimelight) {
      armMotor.set(-armPrPID.calculate(-getAngle(),setpoint));
    }
    if (usingLimelight) {
      System.out.println("now Using Limelight Current offset: "+armLimelight.getLimelightTX());
      armMotor.set(-armPrPID.calculate(-getAngle(),setpoint+(.865*armLimelight.getLimelightTX())));
    }
  }
 
    oldPose = newPose;
    SmartDashboard.putNumber("armMotor Current SPeed", armMotor.get());
    SmartDashboard.putNumber("Desired Pose",setpoint);

    SmartDashboard.putNumber("CURRENT ARM POSE",currentPose);

       armPrPID.setP(SmartDashboard.getNumber("ArmKP", kP));
       armPrPID.setI(SmartDashboard.getNumber("ArmKI", kI));
       armPrPID.setD(SmartDashboard.getNumber("ArmKD", kD));
       kP = SmartDashboard.getNumber("ArmKP", kP);
       kI = SmartDashboard.getNumber("ArmKI", kI);
       kD = SmartDashboard.getNumber("ArmKD", kD);

    // This method will be called once per scheduler run
  }
  public boolean checkLowerLimit(){
    return dLowerLimit.get();
  }
  //sets the arm to a certain pose
  double position;
  public void setPose(double desiredPosition, boolean useLimelight){

    //with the encoder reading 2048 ticks per full rotation, it is 5.68... encoder ticks per degree
      usingLimelight = useLimelight;
      // if(setpoint<desiredPosition){
      //   armPID.setP(kP);
      //   armPID.setI(kI);
      //   armPID.setD(kD);
        
      // }
      // if(setpoint>desiredPosition){
      //   armPID.setP(kPUP);
      //   armPID.setI(kIUP);
      //   armPID.setD(kDUP);
  
      // }
      // if(desiredPosition ==0){
      //   armPID.setP(kPToHome);
      //   armPID.setI(kIToHome);
      //   armPID.setD(kDToHome);
      // }
      
      setpoint = desiredPosition;


    SmartDashboard.putNumber("CURRENT ARM SETPOINT",position);

  }
  //sets winch speed, plan on using operator stick values for this

  //gets current angle 
  public double getAngle(){
    return angEncoder.getDistance();
  }
  //resets encoder
  public void resetEncoder(){
    angEncoder.reset();

  }
  
}