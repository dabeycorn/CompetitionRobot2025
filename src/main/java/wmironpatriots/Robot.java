// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lib.drivers.CommandRobot;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import wmironpatriots.Constants.RobotType;
import wmironpatriots.subsystems.swerve.Swerve;

public class Robot extends CommandRobot {
  public static final RobotType robotType = Robot.isReal() ? RobotType.REAL : RobotType.SIM;

  // HARDWARE
  private final CommandPS5Controller driver = new CommandPS5Controller(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // SUBSYSTEMS
  private final Swerve swerve = Swerve.create();

  // ALERTS
  private final Alert browningOut;

  public Robot() {
    // * SYSTEMS INIT
    // Shuts up driverstation
    DriverStation.silenceJoystickConnectionWarning(true);

    // ! DO NOT REMOVE
    // Signal Logger is set to auto enable when connected to FMS, causing massive delay
    SignalLogger.enableAutoLogging(false);

    // logs build data to the datalog
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Version", BuildConstants.VERSION);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitDirty", String.valueOf(BuildConstants.DIRTY));
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    // Setup Logger data recivers and replay sources
    switch (robotType) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to USB
        Logger.addDataReceiver(new NT4Publisher()); // Log to Network Tables
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher()); // Log to Network Tables only
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull replay file name
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Saves replay as new log
        break;
    }

    // Once Logger starts, no data recivers, replay sources, or metadata can be added
    Logger.start();

    // Sets up alerts
    browningOut = new Alert("Browning Out!", AlertType.kWarning);
    new Trigger(() -> RobotController.isBrownedOut())
        .onTrue(Commands.run(() -> browningOut.set(true)));

    configureBindings();
    configureGameBehavior();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    // Log dashboard info
    SmartDashboard.putNumber("Battery Volts", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("CPU Temps", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL status", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  private void configureBindings() {}

  private void configureGameBehavior() {
    swerve.setDefaultCommand(
        swerve
            .driveFromMagnitudes(
                () -> modifyJoystick(driver.getLeftY() * -1),
                () -> modifyJoystick(driver.getLeftX() * -1),
                () -> modifyJoystick(driver.getRightX() * -1))
            .repeatedly());
  }

  /**
   * Squares and deadbands a joystick value
   *
   * @param val joystick value
   * @return modified joystick value
   */
  private static double modifyJoystick(double val) {
    return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.08);
  }

  /** Command for driver controller rumble */
  private Command rumbleDriver(double value) {
    return Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, value));
  }

  /** Command for driver controller rumble */
  private Command rumbleOperator(double value) {
    return Commands.run(() -> operator.setRumble(RumbleType.kBothRumble, value));
  }

  @Override
  protected Command getAutonCommand() {
    // ! PLACEHOLDER
    return Commands.none();
  }
}
