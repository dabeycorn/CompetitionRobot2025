// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.Swerve.module;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import wmironpatriots.Constants;
import wmironpatriots.subsystems.Swerve.SwerveConstants;
import wmironpatriots.subsystems.Swerve.SwerveConstants.ModuleConfig;

public class ModuleHardwareSim implements ModuleHardware {
  public static final double PIVOT_REDUCTION = 150 / 7;
  public static final double DRIVE_REDUCTION = 6.12;

  private final int index;

  private final DCMotor pivotModel = DCMotor.getKrakenX60Foc(1);
  private final DCMotor driveModel = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim pivotSim, driveSim;
  private double pivotAppliedVolts, driveAppliedVolts;

  private PIDController pivotFeedback = new PIDController(50.0, 0.0, 0.0);
  private PIDController driveFeedback = new PIDController(3.2, 0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.233, 2.02, 0.05);

  public ModuleHardwareSim(ModuleConfig moduleConfig) {
    index = moduleConfig.index();

    pivotSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(pivotModel, 0.004, 150 / 7), pivotModel, 0.0, 0.0);

    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveModel, 0.025, 6.12), driveModel, 0.0, 0.0);

    pivotFeedback.enableContinuousInput(0, 0.5);
  }

  @Override
  public LoggableState getLoggableState() {
    pivotSim.update(Constants.LOOPTIME.in(Seconds));
    driveSim.update(Constants.LOOPTIME.in(Seconds));

    return new LoggableState(
        index,
        true,
        pivotSim.getAngularPositionRotations(),
        0.0,
        pivotAppliedVolts,
        pivotSim.getCurrentDrawAmps(),
        pivotSim.getTorqueNewtonMeters() / pivotModel.KtNMPerAmp,
        true,
        driveSim.getAngularPositionRad() * SwerveConstants.WHEEL_RADIUS.in(Meters),
        driveSim.getAngularVelocityRadPerSec() * SwerveConstants.WHEEL_RADIUS.in(Meters),
        driveFeedback.getSetpoint(),
        driveAppliedVolts,
        driveSim.getCurrentDrawAmps(),
        pivotSim.getTorqueNewtonMeters() / driveModel.KtNMPerAmp,
        true,
        pivotSim.getAngularPositionRotations());
  }

  private double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      motorVoltage = 0.0;
    } else if (motorVoltage > 0.0) {
      motorVoltage -= frictionVoltage;
    } else {
      motorVoltage += frictionVoltage;
    }
    return motorVoltage;
  }

  /**
   * @return the drive motor speed in Meters per Second
   */
  private double getDriveSpeedMps() {
    return (driveSim.getAngularVelocityRPM() / 60)
        * (SwerveConstants.WHEEL_RADIUS.in(Meters) * 2 * Math.PI);
  }

  @Override
  public void setPivotAppliedVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotSim.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  public void setDriveAppliedVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setPivotSetpointPose(double poseRevs) {
    setPivotAppliedVolts(pivotFeedback.calculate(pivotSim.getAngularPositionRotations(), poseRevs));
  }

  @Override
  public void setDriveSetpointSpeed(double speedMps) {
    setDriveAppliedVolts(driveFeedback.calculate(getDriveSpeedMps(), speedMps));
  }

  @Override
  public void stop() {
    pivotSim.setInputVoltage(0.0);
    driveSim.setInputVoltage(0.0);
  }

  @Override
  public void coastingEnabled(boolean enabled) {
    // Simulated motors can't coast lols
  }
}
