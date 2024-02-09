// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.subsystemConstants;
import frc.robot.commands.aprilTagSwerve;
import frc.robot.commands.setHaptics;
import frc.robot.commands.SequentialCommands.fireAndFeed;
import frc.robot.commands.SequentialCommands.spoolShooter;
import frc.robot.commands.SubsystemCommands.setAnalogIntake;
import frc.robot.commands.SubsystemCommands.setArm;
import frc.robot.commands.SubsystemCommands.setIntake;
import frc.robot.commands.SubsystemCommands.setShooter;
import frc.robot.commands.SubsystemCommands.setWinch;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.commands.swervedrive.drivebase.zeroGyroCommand;
import frc.robot.commands.swervedrive.drivebase.*;
import frc.robot.commands.testCommands.setServo;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.controllerHaptics;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.shooterSubsytem;
import frc.robot.subsystems.winchSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.testingSubsystems.servoSubsystem;
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
  intakeSubsystem sIntake  = new intakeSubsystem(Constants.Ports.kIntakeMotorID,
                                                  Constants.Ports.kNoteSensorID);

  shooterSubsytem sShooter = new shooterSubsytem(Constants.Ports.kTopShooterMotorID,
                                                  Constants.Ports.kBotShooterMotorID);

  armSubsystem    sArm     = new armSubsystem(Constants.Ports.kArmMotorID,
                                               Constants.Ports.kLowerLimitID,
                                               Constants.Ports.kArmEncoderID1,
                                               Constants.Ports.kArmEncoderID2);
  winchSubsystem sWinch   = new winchSubsystem(Ports.kWinchMotorID);
  //SwerveDrive swerveDrive= new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve/neo")).createSwerveDrive(Units.feetToMeters(14.5));
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));


  // Configuring Controller inputs and ports
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  XboxController driverXbox = new XboxController(OperatorConstants.kDriverPort);
  XboxController opXbox = new XboxController(OperatorConstants.kDriverPort);  
  private final CommandXboxController m_OpController = new CommandXboxController(1);
    controllerHaptics cHaptics  = new controllerHaptics(driverXbox,opXbox);
// Setting up swerve drive commands as a few options for what we may use

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
  // aprilTagSwerve limelightSwerve = new aprilTagSwerve(drivebase, 
  //                               () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
  //                               OperatorConstants.LEFT_Y_DEADBAND),
  //                               () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
  //                               OperatorConstants.LEFT_X_DEADBAND),
  //                               () -> MathUtil.applyDeadband(driverXbox.getRightX(),
  //                               OperatorConstants.RIGHT_X_DEADBAND),
  //                               () -> driverXbox.getRightBumper());

  //this will be the default command for the intake so we can have manual controll of it                             
  setAnalogIntake analogIntake = new setAnalogIntake(sIntake,
  ()-> MathUtil.applyDeadband(0.1, m_OpController.getLeftTriggerAxis()),
  ()-> MathUtil.applyDeadband(0.1, m_OpController.getRightTriggerAxis()), 
  false);





  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    //Some named commands to be used for auton
    NamedCommands.registerCommand("GroundFeed", new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmGroundFeedPos, false, false).withTimeout(.5),
      new setIntake(sIntake, subsystemConstants.kIntakeSpeed, true),
      new setShooter(sShooter, subsystemConstants.kIdleSpeed, false, false)
      ).withTimeout(0));//Timeout is temp to test auto

    NamedCommands.registerCommand("PrepToShoot",new ParallelCommandGroup(
      new setArm(sArm,subsystemConstants.kArmShootingPos, false, false).withTimeout(.25),
      new setShooter(sShooter, subsystemConstants.kShootingSpeed, false, false)
    ));
    NamedCommands.registerCommand("AlignAndShoot", new SequentialCommandGroup( 
      new ParallelCommandGroup(
    new alignToAprilTag(drivebase,1,false), 
    new setShooter(sShooter, subsystemConstants.kSpoolSpeed, false,false)).withTimeout(.5), 
    new fireAndFeed(sIntake, sShooter)
    ).withTimeout(3));

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
    
    //Driver Bindings


    //Operator Bindings
    m_OpController.leftBumper().whileTrue(new SequentialCommandGroup(new setIntake(sIntake, Constants.subsystemConstants.kIntakeFeedSpeed,true ),
    new setHaptics(cHaptics, 60).withTimeout(.2)));
    m_OpController.rightBumper().onTrue(new SequentialCommandGroup(
      new fireAndFeed(sIntake, sShooter).withTimeout(3.0),
      new setIntake(sIntake, 0, false).withTimeout(.1),
      new setShooter(sShooter, subsystemConstants.kIdleSpeed, false, false)
      ));
    //Turn On intake at kIntakeFeedSpeed until the note sensor reads true
    m_OpController.a().onTrue( new ParallelCommandGroup(
    new setArm(sArm,Constants.subsystemConstants.kArmGroundFeedPos,false,false),
    new setShooter(sShooter, subsystemConstants.kIdleSpeed,false, false))
    );
    m_OpController.povDown().onTrue(new setShooter(sShooter, 0, false, false));
    m_OpController.b().onTrue(new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmShootingPos,false,false),
      new setShooter(sShooter, Constants.subsystemConstants.kSpoolSpeed, false, false)
      ));

    m_OpController.y().onTrue(new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmAmpPos,false,false),
      new setShooter(sShooter, Constants.subsystemConstants.kAmpShootSpeed, false, false)
      ));
    m_OpController.povLeft().whileTrue(new setWinch(sWinch,1.0));
    m_OpController.povRight().whileTrue(new setWinch(sWinch,-1.0));



    m_OpController.x().onTrue(new setArm(sArm,Constants.subsystemConstants.kArmStowPos,false,false));

    //SWERVE BINDINGS

    m_driverController.leftBumper().whileTrue(new aprilTagSwerve(drivebase, 
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                   OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                   OperatorConstants.LEFT_X_DEADBAND),
                                  () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                   OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> driverXbox.getRightBumper(),driverXbox,opXbox));

                                  
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
  public void setIntakeMode()
  {
    sIntake.setDefaultCommand(analogIntake);
  }





}
