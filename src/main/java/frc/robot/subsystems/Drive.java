package frc.robot.subsystems;

// import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.DriveFeedforwards;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private final SparkMax motorLL1,motorRL2,motorLF3,motorRF4;
    private final Encoder LEncoder, REncoder;
    private final Pigeon2 pige;

    private final DifferentialDrive diff;
    private final DifferentialDriveKinematics diffKin;
    private DifferentialDriveOdometry diffOdom;

    public Drive() {
        pige = new Pigeon2(62);

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
        
        LEncoder = new Encoder(2,3,false,EncodingType.k2X);
        REncoder = new Encoder(0,1,true,EncodingType.k2X);

        diff = new DifferentialDrive(motorLL1, motorRL2);
        diff.setSafetyEnabled(false);

        diffKin = new DifferentialDriveKinematics(Units.inchesToMeters(19.5));

        Pose2d start = new Pose2d(0,0,new Rotation2d(0));
        diffOdom = new DifferentialDriveOdometry(pige.getRotation2d(), LEncoder.get(), REncoder.get(),start);
    }
    public void robotCentricDrive(double x, double xr) {
        diff.arcadeDrive(x, xr);
    }
    public void robotCentricTank(double x1, double x2) {
        diff.tankDrive(x1,x2);
    }
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = diffKin.toWheelSpeeds(speeds);
        double leftOutput = wheelSpeeds.leftMetersPerSecond / 4.56;
        double rightOutput = wheelSpeeds.rightMetersPerSecond / 4.56;
        diff.tankDrive(leftOutput,rightOutput);
    }
    public void driveD(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        drive(speeds);
    }
    public double getX() {
        return diffOdom.getPoseMeters().getX();
    }
    public double getY() {
        return diffOdom.getPoseMeters().getY();
    }
    public Rotation2d getH() {
        return pige.getRotation2d();
        // return Rotation2d.fromDegrees(-pige.getYaw().getValueAsDouble());
    }
    public Pose2d getPose() {
        return diffOdom.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        diffOdom.resetPosition(
            getH(),
            LEncoder.getDistance(),
            REncoder.getDistance(),
            pose
        );
    }
    public ChassisSpeeds getChassisSpeeds() {
        return diffKin.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
            LEncoder.getRate(),
            REncoder.getRate()
        ));
    }
    public void resetEncoders() {
        LEncoder.reset();
        REncoder.reset();
    }
    public double getLeftMeters() {
        return LEncoder.getDistance()*Units.inchesToMeters(6)*Math.PI;
    }
    public double getRightMeters() {
        return REncoder.getDistance()*Units.inchesToMeters(6)*Math.PI;
    }

    @Override
    public void periodic() {
        diffOdom.update(pige.getRotation2d(), getLeftMeters(), getRightMeters());
    }

    public double getEncoderLeft() {
        return LEncoder.getDistance();
    }
    public double getEncoderRight() {
        return REncoder.getDistance();
    }
}
