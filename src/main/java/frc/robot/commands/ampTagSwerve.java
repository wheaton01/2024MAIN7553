// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class ampTagSwerve extends Command {
  /** Creates a new ampTagSwerve. */


  Double tx,ty,ta,kP,kI,kD,kPTheta,kITheta,kDTheta,setpoint,thetaSetpoint,tid;
  DoubleSupplier vX,vY,omega;
  BooleanSupplier driveMode;
SwerveSubsystem swerve;
SwerveController controller;
XboxController driverXbox, opXbox;
PIDController thetaController,distanceController;

public ampTagSwerve(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
BooleanSupplier driveMode, XboxController driverXbox, XboxController opXbox)

{
this.driverXbox = driverXbox;
this.opXbox     = opXbox;
this.swerve = swerve;
this.vX = vX;
this.vY = vY;
this.omega = omega;
this.driveMode = driveMode;
this.controller = swerve.getSwerveController();
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(swerve);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = .0825;
    kI= 0.00001;
    kD = 0.000001;
    setpoint=0.0;//this setpoint should be zero and calibrated using limelight itself
   distanceController = new PIDController(kP, kI, kD);
    kPTheta = .015;
    kITheta= 0.00001;
    kDTheta = 0.000001;
    thetaSetpoint=0.0;//this setpoint should be zero and calibrated using limelight itself
   thetaController = new PIDController(kP, kI, kD);
    swerve.limelightOn();
    getLimelightValues();
    System.out.println("LIMELIGHT TRACKING HAS BEGUN!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getLimelightValues();
    double xVelocity   = Math.pow(vX.getAsDouble(), 3);
    double yVelocity   = Math.pow(vY.getAsDouble(), 3);
    double angVelocity = Math.pow(omega.getAsDouble(), 3);
    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);
    printLimelightVal();
    // Drive using raw values.
    if (hasTarget()){
    tid = swerve.getLimelightTID();
    SmartDashboard.putNumber("Current Limelight Target", tid);

      if(tid==7||tid == 4||tid ==1){
      speakerSwerve(xVelocity, yVelocity, angVelocity);
      }
      if(tid==6||tid==5){
      ampSwerve(xVelocity, yVelocity, angVelocity);
      }
 }
 if(!hasTarget()){
      swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 driveMode.getAsBoolean());
 }

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.limelightOff();
    opXbox.setRumble(RumbleType.kBothRumble,0);
    driverXbox.setRumble(RumbleType.kBothRumble,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public void getLimelightValues()
  {
    tx= swerve.getLimelightX();
    ty = swerve.getLimelightY();
    ta = swerve.getLimelightA();
  }
  public double getTy(){
    return swerve.getLimelightY();
  }
  public boolean hasTarget(){
    return swerve.hasTarget();
  }

  public void printLimelightVal()
  {
    getLimelightValues();
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight ty", ty);
    SmartDashboard.putNumber("Limelight ta", ta);
  }
  public void speakerSwerve(double xVelocity,double yVelocity,double angVelocity){
        if (Math.abs(tx)<.1){
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 driveMode.getAsBoolean());
  }
  if (Math.abs(tx)>.1){
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed-(distanceController.calculate(swerve.getLimelightX(), setpoint)), yVelocity* swerve.maximumSpeed),
    angVelocity * controller.config.maxAngularVelocity-(thetaController.calculate(swerve.getLimelightY(),thetaSetpoint)),
    driveMode.getAsBoolean());//may need to do some thinking here as to how i could do both robzot centric driving for apriltags but also able to keep field centric for controls without affecting alignment HMMM
  }
  if (Math.abs(tx)>0.1){
    opXbox.setRumble(RumbleType.kBothRumble,1/(.97*Math.abs(tx)));
    driverXbox.setRumble(RumbleType.kBothRumble,1/(.97*Math.abs(tx)));
  }
  if(Math.abs(tx)<.1){
    opXbox.setRumble(RumbleType.kBothRumble,1.0);
    driverXbox.setRumble(RumbleType.kBothRumble,1.0);
  }
  }

    public void ampSwerve(double xVelocity,double yVelocity,double angVelocity){

    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed-(distanceController.calculate(swerve.getLimelightY(), 0)), yVelocity* swerve.maximumSpeed-.5*(distanceController.calculate(swerve.getLimelightX(), 0))),
    angVelocity -(distanceController.calculate(swerve.getLimelightY(), setpoint))* controller.config.maxAngularVelocity+.5*(thetaController.calculate(swerve.getLimelightY(),thetaSetpoint)),
    driveMode.getAsBoolean());//may need to do some thinking here as to how i could do both robzot centric driving for apriltags but also able to keep field centric for controls without affecting alignment HMMM
 


  if (Math.abs(tx)>0.1){
    opXbox.setRumble(RumbleType.kBothRumble,1/(.97*Math.abs(tx)));
    driverXbox.setRumble(RumbleType.kBothRumble,1/(.97*Math.abs(tx)));
  }
  if(Math.abs(tx)<.1){
    opXbox.setRumble(RumbleType.kBothRumble,1.0);
    driverXbox.setRumble(RumbleType.kBothRumble,1.0);
  }
  }
}
