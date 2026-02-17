package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Launcher extends SubsystemBase {
    // private final SparkMax motorL9, motorL10;
    private final TalonFX motorL9, motorL10;

    public Launcher() {
        motorL9 = new TalonFX(9);
        motorL10 = new TalonFX(10);
        
        motorL9.setNeutralMode(NeutralModeValue.Brake);
        motorL10.setNeutralMode(NeutralModeValue.Brake);

        motorL10.setControl(new Follower(motorL9.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void enable() {
        motorL9.set(0.8);
    }
    public void disable() {
        motorL9.stopMotor();
    }
}
