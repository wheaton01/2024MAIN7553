// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Auton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.aprilTagSwerve;
import frc.robot.commands.testSequence;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.swervedrive.drivebase.zeroGyroCommand;
import frc.robot.commands.testCommands.setServo;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsytem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.testingSubsystems.servoSubsystem;
import swervelib.SwerveDrive;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import swervelib.parser.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //testing a servo out
  servoSubsystem testServo = new servoSubsystem();
//COMMENTED OUT DUE TO CAN ID ERRORS
  // intakeSubsystem sIntake  = new intakeSubsystem(Constants.Ports.kIntakeMotorID,
                                                //  Constants.Ports.kNoteSensorID);
  // shooterSubsytem sShooter = new shooterSubsytem(Constants.Ports.kTopShooterMotorID,
                                                //  Constants.Ports.kBotShooterMotorID);
  // armSubsystem    sArm     = new armSubsystem(Constants.Ports.kArmMotorID, 
                                              // Constants.Ports.kWinchMotorID,
                                              // Constants.Ports.kLowerLimitID,
                                              // Constants.Ports.kArmEncoderID1,
                                              // Constants.Ports.kArmEncoderID2);
// 
// 
  //SwerveDrive swerveDrive= new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve/neo")).createSwerveDrive(Units.feetToMeters(14.5));
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  XboxController driverXbox = new XboxController(OperatorConstants.kDriverPort);

  TeleopDrive teleopDrive = new TeleopDrive(drivebase, 
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                  OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                  OperatorConstants.LEFT_X_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                  OperatorConstants.RIGHT_X_DEADBAND),
                                () -> driverXbox.getRightBumper());

  AbsoluteDrive absDrive = new AbsoluteDrive(drivebase,  
                                () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(1), 0.02),
                                () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(0),0.02),
                                () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(4), 0.02),  
                                () -> -MathUtil.applyDeadband(driverXbox.getRawAxis(5), 0.02));
                            
                            
  AbsoluteFieldDrive teleopField = new AbsoluteFieldDrive(drivebase,   
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0),
                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1),
                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4),
                                OperatorConstants.RIGHT_X_DEADBAND) );
                            
                              //LIMELIGHT DRIVE
  aprilTagSwerve limelightSwerve = new aprilTagSwerve(drivebase, 
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                OperatorConstants.LEFT_X_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                OperatorConstants.RIGHT_X_DEADBAND),
                                () -> driverXbox.getRightBumper());




  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("SetServoPos", new setServo(testServo, .5));
    NamedCommands.registerCommand("SetServoZero", new setServo(testServo, 0));
    NamedCommands.registerCommand("SetServoFull", new setServo(testServo, 1.0));

    configureBindings();

        // Another option that allows you to specify the default auto by its name
        //autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
         autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem)); might be good to use for sequential command control

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_driverController.leftBumper().whileTrue(new aprilTagSwerve(drivebase, 
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                   OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                   OperatorConstants.LEFT_X_DEADBAND),
                                  () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                   OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> driverXbox.getRightBumper()));

                                  
    m_driverController.a().onTrue(new zeroGyroCommand(drivebase));
    
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}


  public void setDriveMode()
  {
   drivebase.setDefaultCommand(absDrive);
    //drivebase.setDefaultCommand();
  }





}
