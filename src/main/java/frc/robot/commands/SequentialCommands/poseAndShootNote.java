// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsytem;
import frc.robot.commands.*;
import frc.robot.commands.SubsystemCommands.setArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class poseAndShootNote extends SequentialCommandGroup {

  intakeSubsystem sIntake;
  armSubsystem    sArm;
  shooterSubsytem sShooter;

  /** Creates a new poseAndShootNote. */
  public poseAndShootNote(intakeSubsystem sIntake, armSubsystem sArm, shooterSubsytem sShooter) {
    this.sArm= sArm;
    this.sIntake = sIntake;
    this.sShooter = sShooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                new setArm(sArm, Constants.subsystemConstants.kArmShootingPos,false,false),
                new spoolShooter(sIntake, sShooter).withTimeout(Constants.subsystemConstants.kWaitForSpool),
                new fireAndFeed(sIntake, sShooter).withTimeout(Constants.subsystemConstants.kWaitForShoot)

    );
  }
}
