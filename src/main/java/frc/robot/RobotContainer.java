  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot;

  import java.io.IOException;
  import java.util.function.BooleanSupplier;

  import org.json.simple.parser.ParseException;

  import com.pathplanner.lib.auto.AutoBuilder;
  import com.pathplanner.lib.config.RobotConfig;
  import com.pathplanner.lib.controllers.PPLTVController;

  import edu.wpi.first.wpilibj.Alert;
  import edu.wpi.first.wpilibj.DriverStation;
  import edu.wpi.first.wpilibj.DriverStation.Alliance;
  import edu.wpi.first.wpilibj.PowerDistribution;
  import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
  import edu.wpi.first.wpilibj.XboxController;
  import edu.wpi.first.wpilibj.Alert.AlertType;
  import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.Command;
  import frc.robot.subsystems.Climb;
  import frc.robot.subsystems.Drive;
  import frc.robot.subsystems.Intake;
  import frc.robot.subsystems.Launcher;
  import frc.robot.subsystems.Omnispike;

  public class RobotContainer {

    private PowerDistribution PD;

    private final Drive driveS;
    private final Intake intakeS;
    private final Launcher launcherS;
    private final Omnispike omnispikeS;
    private final Climb climbS;

    private XboxController driver, operator;

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Integer> driveChooser;
    private int driveType;

    private Alert lowBattery = new Alert("Low Battery!",AlertType.kWarning);
    private Alert replaceBattery = new Alert("Replace Battery Now!",AlertType.kError);

    private double triggerSpeed, leftY, leftX, rightY, rightX;

    public RobotContainer() {
      PD = new PowerDistribution(63,ModuleType.kRev);

      driveS = new Drive();
      intakeS = new Intake();
      launcherS = new Launcher();
      omnispikeS = new Omnispike();
      climbS = new Climb();

      driver = new XboxController(0);
      operator = new XboxController(1);

      configureAutoBuilder();
      configureBindings();

      if (AutoBuilder.isConfigured()) {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser: ", autoChooser);
      } else {
        autoChooser = new SendableChooser<>();
        DriverStation.reportError("AutoBuilder not configured!", false);
      }

      driveChooser = new SendableChooser<>();
      driveChooser.setDefaultOption("Regular (Arcade) Drive", 0);
      driveChooser.addOption("Tank Drive", 1);
      driveChooser.addOption("Rocket League", 2);
      SmartDashboard.putData("Drive Chooser: ", driveChooser);
    }

    private void configureAutoBuilder() {
      RobotConfig robotConfig;
      try {
        robotConfig = RobotConfig.fromGUISettings();
      } catch (IOException | ParseException e) {
        e.printStackTrace();
        DriverStation.reportError("Failed to load robot config: " + e.getMessage(), false);
        return;
      }
      PPLTVController ltvController = new PPLTVController(0.02);
      
      BooleanSupplier shouldFlipPath = () -> {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red;
      };

      AutoBuilder.configure(
        driveS::getPose,
        driveS::resetOdometry,
        driveS::getChassisSpeeds,
        driveS::driveD,
        ltvController,
        robotConfig,
        shouldFlipPath,
        driveS
      );
    }

    private void configureBindings() {}

    public void autoPeriodics() {
      telemetry();
    }

    public void teleopPeriodics() {
      telemetry();

      triggerSpeed = driver.getRightTriggerAxis() - driver.getLeftTriggerAxis();
      leftY = -driver.getLeftY();
      leftX = -driver.getLeftX();
      rightY = -driver.getRightY();
      rightX = -driver.getRightX();
      

      switch (driveType) {
        case 0:
          driveS.robotCentricDrive(leftY, rightX);
          break;
        case 1:
          driveS.robotCentricTank(leftY, rightY);
          break;
        case 2:
          driveS.rocketLeague(triggerSpeed, leftX);
      }
    }

    public void getDriveChoice() {
      driveType = driveChooser.getSelected();
    }

    public void telemetry() {
      SmartDashboard.putNumber("XPos: ", driveS.getX());
      SmartDashboard.putNumber("YPos: ",driveS.getY());
      SmartDashboard.putNumber("Heading: ",driveS.getH().getDegrees());
      SmartDashboard.putNumber("Voltage: ",PD.getVoltage());
      SmartDashboard.putNumber("Encoder Left: ", driveS.getEncoderLeft());
      SmartDashboard.putNumber("Encoder Right: ", driveS.getEncoderRight());
      SmartDashboard.putNumber("Left Meters: ",driveS.getLeftMeters());
      SmartDashboard.putNumber("Right Meters: ",driveS.getRightMeters());
      SmartDashboard.putNumber("Left Y: ", leftY);
      SmartDashboard.putNumber("Right Y: ", rightY);
      SmartDashboard.putNumber("Trigger Speed", triggerSpeed);
      lowBattery.set(PD.getVoltage()<12.0);
      replaceBattery.set(PD.getVoltage() < 11.8);
    }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
  }
