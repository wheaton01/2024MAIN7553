// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelight;

public class armSubsystem extends SubsystemBase {
  /** Creates a new armSubsystem. */
  public double kP, kI, kD, kIz, kFF,kPUP,kDUP,kFFUP,kIUP, kMaxOutput, kMinOutput, maxRPM, oldPose,newPose;
  public double setpoint,currentPose, limelightTa,limelightTx,limelightTy,armScaling;
  public double kPToHome,kIToHome,kDToHome;
  double armOffset;
  int motorID,encoderPort1,encoderPort2,winchMotorID,lowerLimitPort;
  public boolean bGoingUp;
  //Object Creation
  Encoder angEncoder;
  CANSparkMax armMotor;
  DigitalInput dLowerLimit;
  limelight       armLimelight;
  PIDController   armPID;
  PIDController armPrPID,armUpPrPID;
  boolean       usingLimelight,bErrorFlag,bRecoveryMode;
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
    kP = 0.038600;//TODO: TUNE PID HERE
    kI = 0.0;
    kD = 0.0;
    armPrPID = new PIDController(kP,kI, kD);
    // armPrPID = new ProfiledPIDController(kP,kI,kD, 
    // new TrapezoidProfile.Constraints(20,40)) ;
    armPrPID.setTolerance(.25);
    kPUP = 0.034000;//TODO: TUNE PID HERE
    kIUP = 0.0;
    kDUP = 0.0002;
    armUpPrPID = new PIDController(kPUP,kIUP, kDUP);
    // armUpPrPID = new ProfiledPIDController(kPUP,kIUP,kDUP, 
    // new TrapezoidProfile.Constraints(8,15)) ;
    armScaling = .925;
    armUpPrPID.setTolerance(.15);
    armPID = new PIDController(kP, kI,kD);
    
    angEncoder.setDistancePerPulse(360.0/2048.0);
    SmartDashboard.putNumber("ArmKP", kP);
    SmartDashboard.putNumber("ArmKI", kI);
    SmartDashboard.putNumber("ArmKD", kD);
    SmartDashboard.putNumber("ArmKPUP", kPUP);
    SmartDashboard.putNumber("ArmKIUP", kIUP);
    SmartDashboard.putNumber("ArmKDUP", kDUP);
    SmartDashboard.putNumber("armScaling", armScaling);
    armPID.setIZone(kIz);
    armPID.setTolerance(.15);
    armMotor.setVoltage(12.0);
    bErrorFlag = false;
    oldPose= 0;
    newPose= 0;
    bGoingUp=false;
    armOffset = 0;
    bRecoveryMode = false;
  }

  @Override
  public void periodic() {
    //just polls the encoder angle maybe for command stuff idk yet!
    currentPose = angEncoder.getDistance();
    newPose = getAngle();
    SmartDashboard.putNumber("armOffset", armOffset);

    
    if(Math.abs(oldPose)-Math.abs(newPose)>20){
      bErrorFlag = true;
    }
    if (bErrorFlag) {
      armMotor.set(0);
      System.out.println("ARM IS IN ERROR STATE");
    }

if(!bRecoveryMode){
    if(!bErrorFlag){
    if(bGoingUp){
      upPIDControl();
      //armPrPID.setGoal(setpoint);
    }if(!bGoingUp){
      downPIDControl();
      //armUpPrPID.setGoal(setpoint);
    }
  }
    pidSetter();
  }
    SmartDashboard.putNumber("armMotor Current SPeed", armMotor.get());
    SmartDashboard.putNumber("Desired Pose",setpoint);
    SmartDashboard.putNumber("Current Velocity", armMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("CURRENT ARM POSE",currentPose);

    // This method will be called once per scheduler run
  
      oldPose = newPose;


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
        if(Math.abs(setpoint)>Math.abs(desiredPosition)){
          bGoingUp=true;
        }else{
          bGoingUp=false;
        } 

      setpoint = desiredPosition;
    SmartDashboard.putNumber("CURRENT ARM OFFSET", armOffset);
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
  public void pidSetter(){
       armPrPID.setP(SmartDashboard.getNumber("ArmKP", kP));
       armPrPID.setI(SmartDashboard.getNumber("ArmKI", kI));
       armPrPID.setD(SmartDashboard.getNumber("ArmKD", kD));
       armUpPrPID.setP(SmartDashboard.getNumber("ArmKPUP", kPUP));
       armUpPrPID.setI(SmartDashboard.getNumber("ArmKIUP", kIUP));
       armUpPrPID.setD(SmartDashboard.getNumber("ArmKDUP", kDUP));
       kP = SmartDashboard.getNumber("ArmKP", kP);
       kI = SmartDashboard.getNumber("ArmKI", kI);
       kD = SmartDashboard.getNumber("ArmKD", kD);
       
       kPUP = SmartDashboard.getNumber("ArmKPUP", kPUP);
       kIUP = SmartDashboard.getNumber("ArmKIUP", kI);
       kDUP = SmartDashboard.getNumber("ArmKDUP", kD);
       armScaling = SmartDashboard.getNumber("armScaling", armScaling);


  }

  public void downPIDControl(){
      SmartDashboard.putString("ARM PID","DOWN PID IN USE");
      armMotor.setSmartCurrentLimit(30);

     if (!usingLimelight) {
       armMotor.set(armPrPID.calculate(-getAngle(),setpoint));
     }
     if (usingLimelight) {
       SmartDashboard.putNumber("now Using Limelight| Current offset: ",armStableLimelight());
       armMotor.set(armPrPID.calculate(-getAngle(),setpoint+armOffset+(armScaling*armStableLimelight())));
     }
  }
  public void upPIDControl() {
     SmartDashboard.putString("ARM PID","UP PID IN USE");
     armMotor.setSmartCurrentLimit(7);
     if (!usingLimelight) {
       armMotor.set(armUpPrPID.calculate(-getAngle(),setpoint));
     }
     if (usingLimelight) {
       System.out.println("now Using Limelight| Current offset: "+armStableLimelight());
       armMotor.set(armUpPrPID.calculate(-getAngle(),setpoint+armOffset+(armScaling*armStableLimelight())));
     }
  }
  //this is used to stablise the arm at distance to prevent bouncy travel
  double oldLimelight = 0;
  public double armStableLimelight(){
    if(armLimelight.hasTarget()){
      oldLimelight = armLimelight.getLimelightTX();
      return armLimelight.getLimelightTX();
    }else if(!armLimelight.hasTarget()){
      return oldLimelight;
    }
    return 0;
  }
  public void armoffsetUP(){
      armOffset = armOffset+1;

  }
    public void armoffsetDown(){
      armOffset = armOffset-1;
  }
  public void recoveryMode(double setpoint){
    bRecoveryMode = true;
    armMotor.set(setpoint/2);
  }
  public void resetRecoveryMode(){
    bRecoveryMode = false;
  }
}