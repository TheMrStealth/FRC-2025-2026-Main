package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private final SparkMax motorLL1,motorRL2,motorLF3,motorRF4;
    private final RelativeEncoder LEncoder, REncoder;
    private final Pigeon2 gyro;

    private final DifferentialDrive diff;
    private final DifferentialDriveKinematics diffKin;
    private DifferentialDriveOdometry diffOdom;

    public Drive() {
        motorLL1 = new SparkMax(1, MotorType.kBrushless);
        motorLF3 = new SparkMax(3,MotorType.kBrushless);
        motorRL2 = new SparkMax(2, MotorType.kBrushless);
        motorRF4 = new SparkMax(4,MotorType.kBrushless);

        SparkMaxConfig configLL = new SparkMaxConfig();
        configLL.idleMode(IdleMode.kBrake);
        SparkMaxConfig configLF = new SparkMaxConfig();
        configLF.follow(motorLL1);
        configLF.idleMode(IdleMode.kBrake);
        SparkMaxConfig configRL = new SparkMaxConfig();
        configRL.idleMode(IdleMode.kBrake);
        configRL.inverted(true);
        SparkMaxConfig configRF = new SparkMaxConfig();
        configRF.follow(motorRL2);
        configRF.idleMode(IdleMode.kBrake);
        configRF.inverted(true);

        motorLL1.configure(configLL,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorRL2.configure(configRL,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorLF3.configure(configLF,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorRF4.configure(configRF,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        LEncoder = motorLL1.getEncoder();
        REncoder = motorRL2.getEncoder();

        LEncoder.setPosition(0);
        REncoder.setPosition(0);

        diff = new DifferentialDrive(motorLL1, motorRL2);
        diff.setSafetyEnabled(false);

        diffKin = new DifferentialDriveKinematics(Units.inchesToMeters(19.5));

        Pose2d start = new Pose2d(0,0,new Rotation2d(0));
        diffOdom = new DifferentialDriveOdometry(null, LEncoder.getPosition(), REncoder.getPosition(),start);

        gyro = new Pigeon2(0);
    }
    public void robotCentricDrive(double x, double xr) {
        diff.arcadeDrive(x, xr);
    }

    public double getX() {
        return diffOdom.getPoseMeters().getX();
    }
    public double getY() {
        return diffOdom.getPoseMeters().getY();
    }
    public double getH() {
        return diffOdom.getPoseMeters().getRotation().getDegrees();
    }
    public double getHGyro() {
        return gyro.getYaw().getValueAsDouble();
    }

    @Override
    public void periodic() {
        double wheelCircumference = Math.PI * Units.inchesToMeters(6);
        double gearRatio = 8.46;
        double leftPos = (LEncoder.getPosition()/gearRatio) * wheelCircumference;
        double rightPos = (REncoder.getPosition()/gearRatio) * wheelCircumference;
        diffOdom.update(gyro.getRotation2d(), leftPos, rightPos);
        SmartDashboard.putNumber("XPos: ", getX());
        SmartDashboard.putNumber("YPos: ",getY());
        SmartDashboard.putNumber("Heading: ",getH());
        SmartDashboard.putNumber("Heading Gyro: ",getHGyro());  
    }
}
