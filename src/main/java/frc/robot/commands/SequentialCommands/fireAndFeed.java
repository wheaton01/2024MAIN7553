// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsytem;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commands.SubsystemCommands.setIntake;
import frc.robot.commands.SubsystemCommands.setShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class fireAndFeed extends ParallelCommandGroup {
  /** Creates a new zeroIntakeShooter. */
  intakeSubsystem sIntake;
  shooterSubsytem sShooter;
  setIntake cSetIntake;
  public fireAndFeed(intakeSubsystem sIntake, shooterSubsytem sShooter) {

    this.sIntake  = sIntake;
    this.sShooter = sShooter;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new setIntake(sIntake, Constants.subsystemConstants.kIntakeFeedSpeed, false), 
    new setShooter(sShooter, Constants.subsystemConstants.kShootignSpeed, true,false));
  }
}
