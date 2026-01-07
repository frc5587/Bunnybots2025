package frc.robot;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Overrides the driver's rotation input to point the robot in a specific direction.
 */
public class RotateToHeading extends Command {
  private final SwerveSubsystem swerve; 
  private final double targetHeading;
  private final double errorMargin;
  private final double timeMargin;
  
  private Instant lastInstantOutOfRange;
  
  public RotateToHeading(SwerveSubsystem _swerve, double _heading, double _errorMargin, double _timeMargin) {
    swerve = _swerve;
    targetHeading = _heading;
    errorMargin = _errorMargin;
    timeMargin = _timeMargin;
  }
  
  @Override
  public void initialize() {
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
    double secondsElapsed = timeElapsed.getSeconds() + (timeElapsed.getNano()/1000000000.0);

    double currentHeading = swerve.getHeading().getDegrees();
    boolean isInRange = Math.abs(currentHeading - targetHeading) < errorMargin;

    boolean isDisabled = swerve.getIsOverrideHeading() == false;
    
    return (isDisabled)  ||  (isInRange  &&  secondsElapsed < timeMargin);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.deactivateOverrideHeading();
  }
}
