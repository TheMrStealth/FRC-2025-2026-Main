package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Launcher {
    private final SparkMax motorL9, motorL10;

    public Launcher() {
        motorL9 = new SparkMax(10, MotorType.kBrushless);
        motorL10 = new SparkMax(11, MotorType.kBrushless);

        SparkMaxConfig smConfig = new SparkMaxConfig();
        smConfig.idleMode(IdleMode.kBrake);

        motorL9.configure(smConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorL10.configure(smConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void enable() {
        motorL9.set(0.8);
        motorL10.set(0.8);
    }
    public void disable() {
        motorL9.stopMotor();
        motorL10.stopMotor();
    }
}
