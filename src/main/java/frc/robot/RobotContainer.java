// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.indexerAndShoot;
import frc.robot.commands.stopIndexerAndShooter;
import frc.robot.commands.IntakeArmCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Degrees;

import static edu.wpi.first.units.Units.RPM;
import java.io.File;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 // The robot's subsystems and commands are defined here...

   private final IntakeArmSubsystem intaked = new IntakeArmSubsystem();
    private final CommandXboxController driver =
            new CommandXboxController(0);


  private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();

  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
// Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_driverController.getRawAxis(1) * -1, 
      () -> m_driverController.getRawAxis(0) * -1)
      .withControllerRotationAxis(()-> Math.pow(-m_driverController.getRawAxis(4),3)* 0.6)//was 0.125
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.4) // was 0.2
      .allianceRelativeControl(true);


  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -m_driverController.getRawAxis(1),
      () -> -m_driverController.getRawAxis(0))
      .withControllerRotationAxis(() -> Math.pow(-m_driverController.getRawAxis(
          4),3)* 0.125)
      .deadband(0.1)
      .scaleTranslation(0.5)
      .allianceRelativeControl(true);
  // SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
  //     .withControllerHeadingAxis(() -> Math.sin(
  //         m_driverController.getRawAxis(
  //             4) * // was 0
  //             Math.PI)
  //         *
  //         (Math.PI *
  //             2),
  //         () -> Math.cos(
  //             m_driverController.getRawAxis(
  //                 4) * //was 4
  //                 Math.PI)
  //             *
  //             (Math.PI *
  //                 2))
  //     .headingWhile(true)
  //     .translationHeadingOffset(true)
  //     .translationHeadingOffset(Rotation2d.fromDegrees(
  //         0));

  /**                                                                `1d
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Set the default command to force the arm to go to 0.
    

 // Set the default command to force the shooter rest.
    // m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.setDutyCycle(0));

     DriverStation.silenceJoystickConnectionWarning(true);

    // Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // // Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // // Add a simple auto option to have the robot drive forward for 1 second then
    // // stop
    // autoChooser.addOption("Drive Forward",
    // drivebase.driveForward().withTimeout(1));
    // autoChooser.addOption("Drive Forward",
    // drivebase.driveForward().withTimeout(1));

    // // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.setAngle(Degrees.of(0)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    // m_driverController.b().whileTrue(
    // new RunCommand(() -> intake.run(0.8), intake)
    // );

    // m_driverController.a().whileTrue(
    // new RunCommand(() -> intake.run(-0.8), intake)
    // );

    // Move intake to 90 degrees
    m_driverController.a().onTrue(
        Commands.runEnd(
            () -> intaked.moveToAngle(123),
            () -> intaked.stop(),
            intaked));

    m_driverController.b().onTrue(
        Commands.runEnd(
            () -> intaked.moveToAngle(297),
            () -> intaked.stop(),
            intaked));

    m_driverController.rightBumper().whileTrue(new indexerAndShoot(m_ShooterSubsystem, m_IndexerSubsystem))
        .whileFalse(new stopIndexerAndShooter(m_ShooterSubsystem, m_IndexerSubsystem));

    m_driverController.start().whileTrue(m_IndexerSubsystem.run()).whileFalse(m_IndexerSubsystem.stop());

    m_driverController.back().whileTrue(m_IndexerSubsystem.runInverse()).whileFalse(m_IndexerSubsystem.stop());

    m_driverController.leftBumper().whileTrue(m_ShooterSubsystem.setVelocity(-1000))
        .whileFalse(m_ShooterSubsystem.setVelocity(0));
    m_driverController.leftBumper().whileTrue(m_IndexerSubsystem.runInverse()).whileFalse(m_IndexerSubsystem.stop());

    m_driverController.x().toggleOnTrue(m_IntakeSubsystem.setVelocity(-1000));
    m_driverController.y().toggleOnTrue(m_IntakeSubsystem.setVelocity(0));

    m_driverController.povUp()
        .debounce(ShooterConstants.kShooterIncrementPeriod) 
        // this will prolly need tuning to control how quickly the incrments happen
        // *I think* this means that the button will have to be held down at least
        // .5 seconds (or whatever the constant value is) before it fires the command again
        .onTrue(incrementShooterVelocity(ShooterConstants.kShooterUp));

    m_driverController.povDown()
        .debounce(ShooterConstants.kShooterIncrementPeriod) // (see debounce note above on povUp())
        .onTrue(incrementShooterVelocity(ShooterConstants.kShooterDown));

  }
 
  private Command incrementShooterVelocity(double shooterVelocityIncrement) {
    return m_ShooterSubsystem.incrementVelocity(shooterVelocityIncrement);
  }
     
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
