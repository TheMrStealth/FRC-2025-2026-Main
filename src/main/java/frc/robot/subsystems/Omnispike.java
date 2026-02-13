// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// public class Omnispike {
//     private final SparkMax motorSP7, motorSP8;

//     public Omnispike() {
//         motorSP7 = new SparkMax(7, MotorType.kBrushless);
//         motorSP8 = new SparkMax(8, MotorType.kBrushless);

//         SparkMaxConfig motorSP7Config = new SparkMaxConfig();
//         motorSP7Config.idleMode(IdleMode.kBrake);
//         SparkMaxConfig motorSP8Config = new SparkMaxConfig();
//         motorSP8Config.idleMode(IdleMode.kBrake);
//         motorSP8Config.follow(motorSP7);
        
//         motorSP7.configure(motorSP7Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         motorSP8.configure(motorSP8Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }
//     public void enable() {
//         motorSP7.set(0.8);
//     }
//     public void disable() {
//         motorSP8.stopMotor();
//     }
// }
