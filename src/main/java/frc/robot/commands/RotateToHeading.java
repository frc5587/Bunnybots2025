package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Overrides the driver's rotation input to point the robot in a specific direction.
 */
public class RotateToHeading extends Command {
  private final SwerveSubsystem swerve;
  private final DoubleSupplier headingSupplier;

  private double targetHeadingRadians;
  private double lastTimeOutOfRange;
  
  public RotateToHeading(SwerveSubsystem _swerve, DoubleSupplier _headingSupplier) {
    swerve = _swerve;
    headingSupplier = _headingSupplier;
  }
  
  @Override
  public void initialize() {
    targetHeadingRadians = headingSupplier.getAsDouble();
    swerve.overrideHeading(targetHeadingRadians, this);
    lastTimeOutOfRange = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (Math.abs(swerve.getHeading().getDegrees() - targetHeadingRadians) > DrivebaseConstants.HEADING_ERROR_MARGIN_RADIANS) {
      lastTimeOutOfRange = Timer.getFPGATimestamp();
    }
  }

  @Override
  public boolean isFinished() {
    double secondsElapsed = Timer.getFPGATimestamp() - lastTimeOutOfRange;

    double currentHeading = swerve.getHeading().getDegrees();
    boolean isInRange = Math.abs(currentHeading - targetHeadingRadians) < DrivebaseConstants.HEADING_ERROR_MARGIN_RADIANS;

    boolean isDisabled = swerve.getIsOverrideHeading() == false;
    
    return (isDisabled)  ||  (isInRange  &&  secondsElapsed < DrivebaseConstants.HEADING_SECONDS_WITHIN_MARGIN);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.deactivateOverrideHeading();
  }
}
