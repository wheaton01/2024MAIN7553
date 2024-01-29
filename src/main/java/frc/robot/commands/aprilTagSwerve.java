// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class aprilTagSwerve extends Command {
  /** Creates a new aprilTagSwerve. */


  Double tx,ty,ta;
  DoubleSupplier vX,vY,omega;
  BooleanSupplier driveMode;
SwerveSubsystem swerve;
SwerveController controller;
XboxController driverXbox, opXbox;

public aprilTagSwerve(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
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
    if (Math.abs(tx)<1.5){
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 driveMode.getAsBoolean());
  }
  if (Math.abs(tx)>1.){
    swerve.drive(new Translation2d(xVelocity * swerve.maximumSpeed, yVelocity* swerve.maximumSpeed),
    angVelocity+(Constants.Drivebase.limelightSpeed*tx) * controller.config.maxAngularVelocity,//Here im getting a bit fancy trying to incorporate multiple 'axes' into the alignment
    driveMode.getAsBoolean());//may need to do some thinking here as to how i could do both robot centric driving for apriltags but also able to keep field centric for controls without affecting alignment HMMM
  }
  if (Math.abs(tx)>1.){
    opXbox.setRumble(RumbleType.kBothRumble,1/.8*Math.abs(tx));
    driverXbox.setRumble(RumbleType.kBothRumble,1/.8*Math.abs(tx));
  }
  if(Math.abs(tx)<1.){
    opXbox.setRumble(RumbleType.kBothRumble,1.0);
    driverXbox.setRumble(RumbleType.kBothRumble,1.0);
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

  public void printLimelightVal()
  {
    getLimelightValues();
    SmartDashboard.putNumber("Limelight tx", tx);
    SmartDashboard.putNumber("Limelight ty", ty);
    SmartDashboard.putNumber("Limelight ta", ta);
  }
}
