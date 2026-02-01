package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private final SparkMax motorL1,motorR2;
    private final SparkMaxConfig smConfig;
    private final RelativeEncoder LEncoder, REncoder;
    private final Translation2d LwheelLoc, RwheelLoc;

    private final DifferentialDrive diff;
    private final DifferentialDriveKinematics diffKin;
    private DifferentialDriveOdometry diffOdom;

    public Drive() {
        motorL1 = new SparkMax(1, MotorType.kBrushless);
        motorR2 = new SparkMax(2, MotorType.kBrushless);
        
        motorR2.setInverted(true);
        
        smConfig = new SparkMaxConfig();
        smConfig.idleMode(IdleMode.kBrake);

        motorL1.configure(smConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        motorR2.configure(smConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        LEncoder = motorL1.getEncoder();
        REncoder = motorR2.getEncoder();

        LEncoder.setPosition(0);
        REncoder.setPosition(0);

        diff = new DifferentialDrive(motorL1, motorR2);
        diff.setSafetyEnabled(false);

        diffKin = new DifferentialDriveKinematics(Units.inchesToMeters(0)); //need distance between left and right wheel of robot in inches

        LwheelLoc = new Translation2d(0,0);
        RwheelLoc = new Translation2d(0,0);

        Pose2d start = new Pose2d(0,0,new Rotation2d(0));
        diffOdom = new DifferentialDriveOdometry(null, LEncoder.getPosition(), REncoder.getPosition(),start);
    }
    public void robotCentricDrive(double x, double xr) {
        diff.arcadeDrive(x, xr);
    }

    @Override
    public void periodic() {
        double wheelCircumference = Math.PI * Units.inchesToMeters(0);
        double gearRatio = 1;
        double leftPos = (LEncoder.getPosition()/gearRatio) * wheelCircumference;
        double rightPos = (REncoder.getPosition()/gearRatio) * wheelCircumference;
        diffOdom.update(null, leftPos, rightPos);
    }
}
