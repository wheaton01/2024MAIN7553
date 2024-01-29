// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class alignToAprilTag extends Command {
  /** Creates a new aprilTagSwerve. */
  int targetTag;
  Double tx,ty,ta;
  Boolean driveMode;
  SwerveSubsystem swerve;
  SwerveController controller;

public alignToAprilTag(SwerveSubsystem swerve,int targetTag, Boolean driveMode)
{
this.swerve = swerve;
this.targetTag=targetTag;
this.driveMode = driveMode;

this.controller = swerve.getSwerveController();
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(swerve);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    getLimelightValues();
    System.out.println("LIMELIGHT TRACKING HAS BEGUN!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getLimelightValues();


    printLimelightVal();
    // Drive using raw values.
    if (Math.abs(tx)<1.5){
    swerve.drive(new Translation2d(0, 0),
                 0,
                 !driveMode);
  }
  if (Math.abs(tx)>1.5){
    swerve.drive(new Translation2d((0) * swerve.maximumSpeed, (0)* swerve.maximumSpeed),
    (.5*tx) * controller.config.maxAngularVelocity,//Here im getting a bit fancy trying to incorporate multiple 'axes' into the alignment
    !driveMode);//may need to do some thinking here as to how i could do both robot centric driving for apriltags but also able to keep field centric for controls without affecting alignment HMMM
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        swerve.drive(new Translation2d(
                 0,
                 0),
                 0,
                 !driveMode);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(tx>1.5){
      return false;
    }else return true;
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
