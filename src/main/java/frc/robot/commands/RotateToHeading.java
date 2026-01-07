package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
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
  private double lastTimeOutOfRange;
  
  public RotateToHeading(SwerveSubsystem _swerve, DoubleSupplier _headingSupplier, double _errorMargin, double _secondsWithinMargin) {
    swerve = _swerve;
    headingSupplier = _headingSupplier;
    errorMargin = _errorMargin;
    secondsWithinMargin = _secondsWithinMargin;
  }
  
  @Override
  public void initialize() {
    targetHeading = headingSupplier.getAsDouble();
    swerve.overrideHeading(targetHeading, this);
    lastTimeOutOfRange = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if (Math.abs(swerve.getHeading().getDegrees() - targetHeading) > errorMargin) {
      lastTimeOutOfRange = Timer.getFPGATimestamp();
    }
  }

  @Override
  public boolean isFinished() {
    double secondsElapsed = Timer.getFPGATimestamp() - lastTimeOutOfRange;

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
