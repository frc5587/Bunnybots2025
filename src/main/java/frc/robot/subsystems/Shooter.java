package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{

    private final SparkMax shooter = new SparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
    public static SparkMaxConfig shooterConfig = new SparkMaxConfig();
    private double shooterVelocityHigh = ShooterConstants.SHOOTER_VELOCITY_HIGH;
    private double shooterVelocityLow = ShooterConstants.SHOOTER_VELOCITY_LOW;
    public Shooter(){
        super();
        configureShooter();
        // SmartDashboard.putNumber("Shooter Velocity", ShooterConstants.DEFAULT_SHOOTER_VELOCITY);
        // SmartDashboard.putBoolean("Shooter Velocity Change", false);

    }

    public void configureShooter(){
        shooterConfig.inverted(ShooterConstants.SHOOTER_INVERTED);
        shooterConfig.smartCurrentLimit(ShooterConstants.SHOOTER_STALL_LIMIT,ShooterConstants.SHOOTER_FREE_LIMIT);//maybe 
        shooterConfig.idleMode(IdleMode.kCoast);
        shooter.configure(shooterConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }  

    // public double getShuffleboardShooterVelocity(){
    //     double shooterVelocity = SmartDashboard.getNumber("Shooter Velocity", ShooterConstants.DEFAULT_SHOOTER_VELOCITY);
    //     return shooterVelocity;
    // }

    public void shootHigh(){
        shooter.set(shooterVelocityHigh);
    }    

    public void shootLow(){
        shooter.set(shooterVelocityLow);
    }

    public void stop(){
        shooter.set(0);
    }

    @Override
     public void periodic(){     //for testing
    //     if(SmartDashboard.getBoolean("Shooter Velocity Change", false)){
    //         shooterVelocity = SmartDashboard.getNumber("Shooter Velocity", ShooterConstants.DEFAULT_SHOOTER_VELOCITY);
    //     }
    //     SmartDashboard.putBoolean("Shooter Velocity Change", false);
    }
}


