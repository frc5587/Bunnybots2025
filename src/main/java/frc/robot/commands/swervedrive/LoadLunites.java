package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class LoadLunites extends Command {
  public final Intake intake;

  private double startPosition = 0;
  public LoadLunites (Intake intakeInput) {
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
}
