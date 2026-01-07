import java.time.Duration;
import java.time.Instant;
import java.util.concurrent.TimeUnit;

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
    if (Math.abs(currentHeading - targetHeading) > errorMargin) {
      lastInstantOutOfRange = Instant.now();
    }
  }

  @Override
  public boolean isFinished() {
    double currentHeading = swerve.getHeading(); // Psuedocode
    Duration timeElapsed = Duration.between(lastInstantOutOfRange, Instant.now());
    boolean isInRange = Math.abs(currentHeading - targetHeading) < errorMargin;

    boolean isDisabled = swerve.getIsOverrideHeading() == false;
    
    return (isDisabled)  ||  (isInRange  &&  timeElapsed < timeMargin);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.deactivateOverrideHeading();
  }
}
