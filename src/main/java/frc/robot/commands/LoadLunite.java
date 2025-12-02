package frc.robot.commands;

import java.nio.file.WatchEvent;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MidstageConstants;
import frc.robot.subsystems.Midstage;

public class LoadLunite extends Command {
  private final Midstage midstage;
  private final BooleanSupplier shooterReady;

  private double startPosition = 0;

  public LoadLunite (Midstage midstageInput) {
    midstage = midstageInput;
    shooterReady = () -> true;
  }

  public LoadLunite(Midstage midstageInput, BooleanSupplier isReadySupplier) {
    midstage = midstageInput;
    shooterReady = isReadySupplier;
  }

  @Override
  public void initialize() {
    startPosition = midstage.getPosition();
  }

  @Override
  public void execute() {
    if (!shooterReady.getAsBoolean()) {
      midstage.start();
    }
  }

  @Override
  public boolean isFinished() {
    return !midstage.hasLunite();
  }

  @Override
  public void end(boolean interrupted) {
    midstage.stop();
  }
}
