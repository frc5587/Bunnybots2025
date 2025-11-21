package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class LoadLunite extends Command {
  public final Intake intake;

  private double startPosition = 0;

  public LoadLunite (Intake intakeInput) {
    intake = intakeInput;
  }

  @Override
  public void initialize() {
    intake.start();
    startPosition = intake.getPosition();
  }

  @Override
  public boolean isFinished() {
    return startPosition - intake.getPosition() < IntakeConstants.LOAD_DISTANCE;
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }
}
