package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class LoadLunites extends Command {
  public final Intake intake;
  public final int numberOfLunites;

  private double startPosition = 0;

  public LoadLunites (Intake intakeInput, int lunites) {
    intake = intakeInput;
    numberOfLunites = lunites;
  }

  @Override
  public void initialize() {
    intake.start();
    startPosition = intake.getPosition();
  }

  @Override
  public boolean isFinished() {
    return startPosition - intake.getPosition() < IntakeConstants.LOAD_DISTANCE * numberOfLunites;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
