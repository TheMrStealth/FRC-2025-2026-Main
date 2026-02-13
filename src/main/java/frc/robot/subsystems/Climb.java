// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// public class Climb {
//     private final SparkMax motorC11, motorC12;

//     public Climb() {
//         motorC11 = new SparkMax(11, MotorType.kBrushless);
//         motorC12 = new SparkMax(12, MotorType.kBrushless);

//         SparkMaxConfig motorC11Config = new SparkMaxConfig();
//         motorC11Config.idleMode(IdleMode.kBrake);
//         SparkMaxConfig motorC12Config = new SparkMaxConfig();
//         motorC12Config.idleMode(IdleMode.kBrake);
//         motorC12Config.follow(motorC11);

//         motorC11.configure(motorC11Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         motorC12.configure(motorC12Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }
// }
