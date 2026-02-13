// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// public class Intake {
//     private final SparkMax motorI5, motorV6;

//     public Intake() {
//         motorI5 = new SparkMax(5, MotorType.kBrushless);
//         motorV6 = new SparkMax(6,MotorType.kBrushless);

//         SparkMaxConfig motorI5Config = new SparkMaxConfig();
//         motorI5Config.idleMode(IdleMode.kBrake);
//         SparkMaxConfig motorV6Config = new SparkMaxConfig();
//         motorV6Config.idleMode(IdleMode.kBrake);
//         motorV6Config.follow(motorI5);

//         motorI5.configure(motorI5Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         motorV6.configure(motorV6Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }
//     public void forward() {
//         motorI5.set(0.8);
//     }
//     public void reverse() {
//         motorI5.set(-0.8);
//     }
//     public void disable() {
//         motorI5.stopMotor();
//     }
// }
