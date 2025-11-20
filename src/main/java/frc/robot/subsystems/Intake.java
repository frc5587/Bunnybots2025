package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax intake = new SparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);
  public static SparkMaxConfig intakeConfig = new SparkMaxConfig();

  public Intake() {
    super();
    intakeConfig.inverted(IntakeConstants.INTAKE_INVERTED);
    intakeConfig.smartCurrentLimit(IntakeConstants.INTAKE_STALL_LIMIT, IntakeConstants.INTAKE_FREE_LIMIT); // maybe
    intakeConfig.idleMode(IdleMode.kCoast);
    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    intake.set(0);
  }

  public void start() {
    intake.set(IntakeConstants.INTAKE_SPEED);
  }

  public double getPosition() {
    return intake.getAbsoluteEncoder().getPosition();
  }
}
