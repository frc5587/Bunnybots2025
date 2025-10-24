package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private SparkMax shooter = new SparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
        private ProfiledPIDController shooterPID;
        public static SparkMaxConfig shooterConfig = new SparkMaxConfig();
        public Shooter(){
            super();
            configureShooter();
            this.shooterPID=ShooterConstants.SHOOTER_PID;
    }
    public void configureShooter(){
        shooterConfig.inverted(ShooterConstants.SHOOTER_INVERTED);
        shooterConfig.smartCurrentLimit(30,25);//maybe 
        shooterConfig.idleMode(IdleMode.kCoast);
        shooter.configure(shooterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }  
    public void setShooterVoltage(double voltage){
        shooter.set(voltage / RobotController.getBatteryVoltage());
    }
    /*
    public double getShooterMPS(){
        return(shooter.getEncoder().getVelocity() /60) *ShooterConstants.WHEEL_CIRCUMFERENCE;
    }
    public void setShooterSpeed(double shooterSpeed){
        shooter.setShooterVoltage(shooterPID.calculate(g
    }
    public void */

    
}


