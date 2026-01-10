// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MidstageConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.RotateToHeading;
import frc.robot.subsystems.Midstage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Shooter shooter = new Shooter();
  private final Midstage midstage = new Midstage();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  
  private SendableChooser<Command> autoChooser;
  private boolean maintainHeading = false;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveFieldOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true)
      .robotRelative(false);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveFieldOriented.copy().robotRelative(true)
      .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle =
  driveFieldOriented.copy().withControllerHeadingAxis(driverXbox::getRightX,
  driverXbox::getRightY)
  .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Register Named Commands
    NamedCommands.registerCommand("shooterHighGoal", Commands.parallel(Commands.runOnce(shooter::shootHigh, shooter),
                                                                            Commands.waitSeconds(ShooterConstants.SPINUP_WAIT_TIME)));
    NamedCommands.registerCommand("shooterLowGoal", Commands.runOnce(shooter::shootLow, shooter));
    NamedCommands.registerCommand("shooterStop", Commands.runOnce(shooter::stop, shooter));
    NamedCommands.registerCommand("loadLunites", Commands.parallel(Commands.runOnce(midstage::start),
                                                                        (Commands.waitSeconds(MidstageConstants.LOAD_TIME)))
                                                                        .andThen(Commands.runOnce(midstage::stop)));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Swerve
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveFieldOriented);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } 
    else if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
    }
    else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // this is the main drive command
    }

    // Rotate 90 degrees, aligning with the field
    Command turnRight = new RotateToHeading(drivebase, () -> Math.round(drivebase.getIdealHeadingRadians()) * 90 + 90, 2, 0.5);
    Command turnLeft = new RotateToHeading(drivebase, () -> Math.round(drivebase.getIdealHeadingRadians()) * 90 - 90, 2, 0.5);
    Command turnRightAndHold = Commands.runOnce(() -> {drivebase.overrideHeading(Math.round(drivebase.getIdealHeadingRadians()) * 90.0 + 90, null);});
    Command turnLeftAndHold = Commands.runOnce(() -> {drivebase.overrideHeading(Math.round(drivebase.getIdealHeadingRadians()) * 90.0 - 90, null);});
    driverXbox.rightBumper().onTrue(maintainHeading ? turnRightAndHold : turnRight);
    driverXbox.leftBumper().onTrue(maintainHeading ? turnLeftAndHold : turnLeft);

    // Dpad to rotate robot
    driverXbox.povUp().onTrue(new RotateToHeading(drivebase, () -> 0., 5, 0.5));
    driverXbox.povRight().onTrue(new RotateToHeading(drivebase, () -> Math.PI / -2., 5, 0.5));
    driverXbox.povDown().onTrue(new RotateToHeading(drivebase, () -> -1. * Math.PI, 5, 0.5));
    driverXbox.povLeft().onTrue(new RotateToHeading(drivebase, () -> Math.PI / 2., 5, 0.5));

    // Holds the current heading when maintainHeading is toggled on
    driverXbox.rightStick().onTrue(Commands.runOnce(() -> {
        maintainHeading = !maintainHeading;
        if (maintainHeading) {
          drivebase.overrideHeading(drivebase.getIdealHeadingRadians(), null);
        }
        else {
          drivebase.deactivateOverrideHeading();
        }}));

    // Stuff
    driverXbox.b().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.a().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Slow mode
    driverXbox.leftTrigger().whileTrue(Commands.runEnd(
        () -> driveFieldOriented.scaleTranslation(.6).scaleRotation(.5), 
        () -> driveFieldOriented.scaleTranslation(.8).scaleRotation(1.)));

    // Reverse
    operatorXbox.leftTrigger().whileTrue(Commands.runOnce(shooter::reverse).alongWith(Commands.runOnce(midstage::reverse)))
                              .onFalse(Commands.runOnce(shooter::stop).alongWith(Commands.runOnce(midstage::stop)));

    // Shooter hold
    operatorXbox.rightBumper().whileTrue(Commands.runOnce(shooter::shootHigh))
                              .onFalse(Commands.runOnce(shooter::stop)); // High speed shoot
    operatorXbox.rightTrigger().whileTrue(Commands.runOnce(shooter::shootLow))
                               .onFalse(Commands.runOnce(shooter::stop)); // Low speed shoot
    
    // Shooter press once
    // operatorXbox.rightBumper().onTrue(Commands.runOnce(shooter::shootHigh)); // High speed shoot
    // operatorXbox.rightTrigger().onTrue(Commands.runOnce(shooter::shootLow)); // Low speed shoot
    // operatorXbox.x().onTrue(Commands.runOnce(shooter::stop));
    // operatorXbox.b().onTrue(Commands.runOnce(shooter::stop));

    // Midstage
    operatorXbox.leftBumper().whileTrue(Commands.runOnce(midstage::start))
                             .onFalse(Commands.runOnce(midstage::stop)); // Spin midstage while pressed
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

  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }
}
