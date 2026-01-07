package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MidstageConstants;

public class Midstage extends SubsystemBase {

  private final SparkMax motor = new SparkMax(MidstageConstants.MIDSTAGE_ID, MotorType.kBrushless);
  // private final DigitalInput beamBreak = new DigitalInput(MidstageConstants.BEAMBREAK_ID);
  public static SparkMaxConfig motorConfig = new SparkMaxConfig();

  public Midstage() {
    super();
    SmartDashboard.putNumber("Midstage Position", getPosition());

    motorConfig.inverted(MidstageConstants.MIDSTAGE_INVERTED);
    motorConfig.smartCurrentLimit(MidstageConstants.MIDSTAGE_STALL_LIMIT, MidstageConstants.MIDSTAGE_FREE_LIMIT); // maybe
    motorConfig.idleMode(IdleMode.kBrake);
    motorConfig.encoder.positionConversionFactor(MidstageConstants.POSITION_CONVERSION_FACTOR);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    motor.set(0);
  }

  public void start() {
    motor.set(MidstageConstants.MIDSTAGE_SPEED);
  }

  public void reverse() {
    motor.set(-1*MidstageConstants.MIDSTAGE_REVERSE_SPEED);
  }

  public double getPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Midstage Position", getPosition());
  }

  // public boolean hasLunite() {
  //   return !beamBreak.get();
  // }
}
