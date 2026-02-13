// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// public class Launcher {
//     private final SparkMax motorL9, motorL10;

//     public Launcher() {
//         motorL9 = new SparkMax(9, MotorType.kBrushless);
//         motorL10 = new SparkMax(10, MotorType.kBrushless);

//         SparkMaxConfig motorL9Config = new SparkMaxConfig();
//         motorL9Config.idleMode(IdleMode.kBrake);
//         SparkMaxConfig motorL10Config = new SparkMaxConfig();
//         motorL10Config.idleMode(IdleMode.kBrake);
//         motorL10Config.follow(motorL9);

//         motorL9.configure(motorL9Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         motorL10.configure(motorL10Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     public void enable() {
//         motorL9.set(0.8);
//     }
//     public void disable() {
//         motorL9.stopMotor();
//     }
// }
