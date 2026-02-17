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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
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

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser: ", autoChooser);
  }

  private void configureAutoBuilder() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      return;
    }
    PPLTVController ltvController = new PPLTVController(2.0);
    
    BooleanSupplier shouldFlipPath = () -> {
      var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
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

    driveS.robotCentricDrive(-driver.getLeftY(), -driver.getRightX());
    // driveS.robotCentricTank(-driver.getLeftY(), -driver.getRightY());
  }

  public void telemetry() {
    SmartDashboard.putNumber("XPos: ", driveS.getX());
    SmartDashboard.putNumber("YPos: ",driveS.getY());
    SmartDashboard.putNumber("Heading: ",driveS.getH().getDegrees());
    SmartDashboard.putNumber("Voltage: ",PD.getVoltage());
    SmartDashboard.putNumber("Encoder Left: ", driveS.getEncoderLeft());
    SmartDashboard.putNumber("Encoder Right: ", driveS.getEncoderRight());
    SmartDashboard.putNumber("Left Y: ", -driver.getLeftY());
    SmartDashboard.putNumber("Right Y: ", -driver.getRightY());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
