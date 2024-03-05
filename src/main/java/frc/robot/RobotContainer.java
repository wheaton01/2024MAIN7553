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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivebase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.subsystemConstants;
import frc.robot.commands.ampTagSwerve;
import frc.robot.commands.aprilTagSwerve;
import frc.robot.commands.setHaptics;
import frc.robot.commands.SequentialCommands.fireAndFeed;
import frc.robot.commands.SequentialCommands.spoolShooter;
import frc.robot.commands.SubsystemCommands.armRecovery;
import frc.robot.commands.SubsystemCommands.setAnalogIntake;
import frc.robot.commands.SubsystemCommands.setArm;
import frc.robot.commands.SubsystemCommands.setIntake;
import frc.robot.commands.SubsystemCommands.setShooter;
import frc.robot.commands.SubsystemCommands.setWinch;
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
  XboxController opXbox = new XboxController(OperatorConstants.kOperatorPort);  
  private final CommandXboxController m_OpController = new CommandXboxController(1);
    controllerHaptics cHaptics  = new controllerHaptics(driverXbox,opXbox);
// Setting up swerve drive commands as a few options for what we may use

  TeleopDrive teleopDrive = new TeleopDrive(drivebase, 
                                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                  OperatorConstants.LEFT_Y_DEADBAND),
                                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                  OperatorConstants.LEFT_X_DEADBAND),
                                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                  OperatorConstants.RIGHT_X_DEADBAND),
                                () -> driverXbox.getRightBumper());

  AbsoluteDrive absDrive = new AbsoluteDrive(drivebase,  
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), 0.02),
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0),0.02),
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(4), 0.02),  
                                () -> MathUtil.applyDeadband(driverXbox.getRawAxis(5), 0.02));
                            
                            
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
  ()-> MathUtil.applyDeadband( m_OpController.getLeftTriggerAxis(), .5),
  ()-> MathUtil.applyDeadband(m_OpController.getRightTriggerAxis(), .5), 
  false);

  setWinch      analogSetWinch = new setWinch(sWinch,
  ()-> -MathUtil.applyDeadband(m_OpController.getRawAxis(5),.25));

  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    //Some named commands to be used for auton
    NamedCommands.registerCommand("GroundFeed", new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmGroundFeedPos, false, false).withTimeout(.250),
      new setIntake(sIntake, subsystemConstants.kIntakeSpeed, true,true),
      new setShooter(sShooter, subsystemConstants.kIdleSpeed, false, false)
    ));//Timeout is temp to test auto

    NamedCommands.registerCommand("AlignToTarget",
    new alignToAprilTag(drivebase,1,false).withTimeout(2.0));
    NamedCommands.registerCommand("PrepToShoot",new ParallelCommandGroup(
      new setArm(sArm,subsystemConstants.kArmShootingPos, false, true).withTimeout(.5),
      new setShooter(sShooter, subsystemConstants.kShootingSpeed, false, false)
    ));
    NamedCommands.registerCommand("AlignAndShoot", new SequentialCommandGroup( 
      new ParallelCommandGroup(
      new alignToAprilTag(drivebase,1,false), 
      new setArm(sArm,subsystemConstants.kArmShootingPos, false, true).withTimeout(.5),
      new setShooter(sShooter, subsystemConstants.kShootingSpeed, false,false).withTimeout(2)
      ), 
      new fireAndFeed(sIntake, sShooter).withTimeout(.5)));

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
    //Operator Bindings
    // m_driverController.rightBumper().whileTrue(new SequentialCommandGroup(new setIntake(sIntake, Constants.subsystemConstants.kIntakeSpeed,true,false ),
    // new setHaptics(cHaptics, 60).withTimeout(.2)));
    m_OpController.leftBumper().whileTrue(new SequentialCommandGroup(new setIntake(sIntake, Constants.subsystemConstants.kIntakeSpeed,true,false),
    new setHaptics(cHaptics, 60).withTimeout(.2)));

    m_OpController.rightBumper().onTrue(new SequentialCommandGroup(
    new setShooter(sShooter, subsystemConstants.kShootingSpeed, false, false),   
    new setIntake(sIntake, subsystemConstants.kIntakeFeedSpeed, false,false).withTimeout(1.0),
    new setIntake(sIntake, 0, false,false).withTimeout(.01),
    new setShooter(sShooter, subsystemConstants.kIdleSpeed, false, false).withTimeout(.15)
      ));
    //Turn On intake at kIntakeFeedSpeed until the note sensor reads true
    m_OpController.a().onTrue( new ParallelCommandGroup(
    new setArm(sArm,Constants.subsystemConstants.kArmGroundFeedPos,false,false),
    new setShooter(sShooter, subsystemConstants.kIdleSpeed,false, false))
    );

    m_OpController.povLeft().onTrue(new setShooter(sShooter, 0, false, false));
    
    m_OpController.b().onTrue(new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmShootingPos,false,true),
      new setShooter(sShooter, Constants.subsystemConstants.kSpoolSpeed, false, false)
      ));

    m_OpController.y().onTrue(new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmAmpPos,false,false),
      new setShooter(sShooter, Constants.subsystemConstants.kIdleSpeed, false, false)
      ));


    m_OpController.x().onTrue(new ParallelCommandGroup(
      new setArm(sArm,subsystemConstants.kArmStowPos,false,false),
      new setShooter(sShooter,subsystemConstants.kShootingSpeed, false, false)));

    m_OpController.povDown().onTrue(new InstantCommand(sArm::armoffsetDown));
    m_OpController.povUp().onTrue(new InstantCommand(sArm::armoffsetUP));
    m_OpController.button(9).onTrue(new armRecovery(sArm,()-> m_OpController.getRawAxis(1)));
    //SWERVE BINDINGS

    m_driverController.leftBumper().whileTrue(new aprilTagSwerve(drivebase, 
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                   OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                   OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                   OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> !driverXbox.getRightBumper(),driverXbox,opXbox));

    m_driverController.rightBumper().whileTrue(new ampTagSwerve(drivebase, 
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                   OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                   OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                   OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> !driverXbox.getRightBumper(),driverXbox,opXbox));

                                  
    m_driverController.a().onTrue(new zeroGyroCommand(drivebase));

    m_driverController.b().onTrue( new ParallelCommandGroup(
    new setArm(sArm,Constants.subsystemConstants.kArmStowPos,false,false),
    new setShooter(sShooter, subsystemConstants.kIdleSpeed,false, false))
    );
    m_driverController.y().onTrue(new ParallelCommandGroup(
      new setArm(sArm,Constants.subsystemConstants.kArmAmpPos,false,false),
      new setShooter(sShooter, Constants.subsystemConstants.kIdleSpeed, false, false)
      ));
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
    sWinch.setDefaultCommand(analogSetWinch);
  }
  public void updatePose(){
    //new InstantCommand(drivebase::addFakeVisionReading);
  }
  public void startCameraServer(){
    
  }
  public void resetAutonMode(){
    new InstantCommand(sIntake::resetAutonMode);
  }

}
