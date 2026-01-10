// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MidstageConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Midstage;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final Shooter shooter = new Shooter();
  private final Midstage midstage = new Midstage();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private SendableChooser<Command> autoChooser;
  private Command loadLunite = Commands.parallel(Commands.runOnce(midstage::start),
                                                                       Commands.waitSeconds(0.1))
                                                     .andThen(Commands.parallel(Commands.runOnce(midstage::stop),
                                                                                Commands.waitSeconds(0.5)))
                                                     .andThen(Commands.parallel(Commands.runOnce(midstage::reverse),
                                                                                Commands.runOnce(shooter::reverse),
                                                                                Commands.waitSeconds(0.3)))
                                                     .andThen(Commands.parallel(Commands.runOnce(midstage::stop),
                                                                                Commands.waitSeconds(0.5)));
  private Command shooterHigh = Commands.parallel(Commands.runOnce(shooter::shootHigh, shooter),
                                                  Commands.waitSeconds(ShooterConstants.SPINUP_WAIT_TIME));
  private Command shooterLow = Commands.parallel(Commands.runOnce(shooter::shootLow, shooter),
                                                  Commands.waitSeconds(ShooterConstants.SPINUP_WAIT_TIME));
  private Command shooterStop = Commands.runOnce(shooter::stop, shooter);
  // private final LEDController ledController = new LEDController();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream fieldOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1, 
                                                                /* 
                                                                TODO this is a sketchy fix to the "front is back" issue, 
                                                                if we are getting weird problems this may be the culprit
                                                                -1's were changed to 1's
                                                                */
                                                                () -> driverXbox.getLeftX() * 1)
                                                            .withControllerRotationAxis(()->driverXbox.getRightX())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true)
                                                            .robotRelative(false);
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream robotOriented = fieldOriented.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream directAngle = fieldOriented.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Register Named Commands
    NamedCommands.registerCommand("shooterHighGoal", shooterHigh);
    NamedCommands.registerCommand("shooterLowGoal", shooterLow);
    NamedCommands.registerCommand("shooterStop", shooterStop);
    // NamedCommands.registerCommand("shootLow",  Commands.sequence(shooterLow,
    //                                                                   loadLunite,shooterLow,
    //                                                                   loadLunite,shooterLow,
    //                                                                   loadLunite,shooterLow,
    //                                                                   loadLunite,shooterStop));
    
    // NamedCommands.registerCommand("shootHigh", Commands.sequence(shooterLow,
    //                                                                   loadLunite, shooterLow,
    //                                                                   loadLunite, shooterLow,
    //                                                                   loadLunite, shooterLow,
    //                                                                   loadLunite, shooterStop));
    NamedCommands.registerCommand("startMidstage",Commands.runOnce(midstage::start));
    NamedCommands.registerCommand("stopMidstage", Commands.runOnce(midstage::stop));
    NamedCommands.registerCommand("loadLunite", loadLunite);
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
  private void configureBindings()
  {
    Command driveDirectAngle = drivebase.driveFieldOriented(directAngle);
    Command driveFieldOriented = drivebase.driveFieldOriented(fieldOriented);
    Command driveRobotOriented  = drivebase.driveFieldOriented(robotOriented);

      drivebase.setDefaultCommand(driveFieldOriented); // this is the main drive command

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOriented); // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
    }
    // Driver extra controls
    driverXbox.b().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.a().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Slow mode
    driverXbox.leftTrigger().whileTrue(Commands.runEnd(
        () -> fieldOriented.scaleTranslation(.5).scaleRotation(.5), 
        () -> fieldOriented.scaleTranslation(.8).scaleRotation(1.)
        ));

    // Reverse
    operatorXbox.leftTrigger().whileTrue(Commands.runOnce(shooter::reverse).alongWith(Commands.runOnce(midstage::reverse)))
                    .onFalse(Commands.runOnce(shooter::stop).alongWith(Commands.runOnce(midstage::stop)));

    // Shooter
    // Hold
    operatorXbox.rightBumper().whileTrue(Commands.runOnce(shooter::shootHigh))
                              .onFalse(Commands.runOnce(shooter::stop)); // High speed shoot
    operatorXbox.rightTrigger().whileTrue(Commands.runOnce(shooter::shootLow))
                               .onFalse(Commands.runOnce(shooter::stop)); // Low speed shoot
    // Press once
    operatorXbox.rightBumper().onTrue(Commands.runOnce(shooter::shootHigh)); // High speed shoot
    operatorXbox.rightTrigger().onTrue(Commands.runOnce(shooter::shootLow)); // Low speed shoot
    operatorXbox.x().onTrue(Commands.runOnce(shooter::stop));
    operatorXbox.b().onTrue(Commands.runOnce(shooter::stop));

    // Midstage
    operatorXbox.leftBumper().whileTrue(Commands.runOnce(midstage::start))
        .onFalse(Commands.runOnce(midstage::stop)); // Spin midstage while pressed
        

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    // drivebase.setMotorBrake(brake);
  }
}
