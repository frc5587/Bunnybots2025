package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Overrides the driver's rotation input to point the robot in a specific direction.
 */
public class RotateToHeading extends Command {
  private final SwerveSubsystem swerve;
  private final DoubleSupplier headingSupplier;
  private final double errorMargin;
  private final double secondsWithinMargin;

  private double targetHeading;
  
  private Instant lastInstantOutOfRange;
  
  public RotateToHeading(SwerveSubsystem _swerve, DoubleSupplier _headingSupplier, double _errorMargin, double _secondsWithinMargin) {
    swerve = _swerve;
    headingSupplier = _headingSupplier;
    errorMargin = _errorMargin;
    secondsWithinMargin = _secondsWithinMargin;
  }
  
  @Override
  public void initialize() {
    targetHeading = headingSupplier.getAsDouble();
    swerve.overrideHeading(targetHeading);
    lastInstantOutOfRange = Instant.now();
  }

  @Override
  public void execute() {
    if (Math.abs(swerve.getHeading().getDegrees() - targetHeading) > errorMargin) {
      lastInstantOutOfRange = Instant.now();
    }
  }

  @Override
  public boolean isFinished() {
    Duration timeElapsed = Duration.between(lastInstantOutOfRange, Instant.now());
    double secondsElapsed = timeElapsed.toNanos()/1000000000.0;

    double currentHeading = swerve.getHeading().getDegrees();
    boolean isInRange = Math.abs(currentHeading - targetHeading) < errorMargin;

    boolean isDisabled = swerve.getIsOverrideHeading() == false;
    
    return (isDisabled)  ||  (isInRange  &&  secondsElapsed < secondsWithinMargin);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.deactivateOverrideHeading();
  }
}
