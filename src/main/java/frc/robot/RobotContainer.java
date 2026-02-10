// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class RobotContainer {

  private final Drive driveS;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    driveS = new Drive();
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

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
