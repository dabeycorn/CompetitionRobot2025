// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;

/** Generalized hardware methods for a swerve-module's hardware */
public interface ModuleHardware {
  /**
   * Represents a set of measured values from module hardware
   *
   * <p> index
   * <p> pivotIsOk Is pivot motor measuring correctly?
   * <p> pivotRevs Position measurement of pivot motor in revolutions (relative encoder)
   * <p> pivotSetpointPoseRevs Position setpoint of pivot motor in revolutions
   * <p> pivotAppliedVolts Output voltage of pivot motor in volts
   * <p> pivotStatorAmps Stator current of pivot motor in amps
   * <p> pivotTorqueAmps Torque output of pivot motor in amps
   * <p> driveIsOk Is drive motor measuring correctly?
   * <p> driveDistance Distance driven in meters
   * <p> driveMps Speed measurement of the drive motor in Meters/Second
   * <p> driveSetpointMps Speed setpoint of drive motor in Meters/Second
   * <p> driveAppliedVolts Output voltage of drive motor in volts
   * <p> driveStatorAmps Stator current of drive motor in amps
   * <p> driveTorqueAmps Torque output of drive motor in amps
   * <p> cancoderIsOk Is CANcoder measuring correctly?
   * <p> cancoderRevs Position measurement of pivot motor in revolutions
   */
  @AutoLog
  public static class LoggableState {
    public int index;

    public boolean pivotIsOk = false;
    public double pivotRevs;
    public double pivotSetpointRevs;
    public double pivotAppliedVolts;
    public double pivotCurrentAmps;
    public double pivotTorqueAmps;

    public boolean driveIsOk = false;
    public double driveDistanceMeters;
    public double driveMps;
    public double driveSetpointMps;
    public double driveAppliedVolts;
    public double driveCurrentAmps;
    public double driveTorqueAmps;

    public boolean cancoderIsOk = false;
    public double cancoderRevs;
  }

  /**
   * Updates an older {@link LoggableState} with new measurements
   */
  public void updateLoggableState(LoggableState oldState);

  /**
   * Set pivot motor setpoint voltage
   *
   * <p> volts Desired voltage
   */
  public void setPivotAppliedVolts(double volts);

  /**
   * Set drive motor setpoint voltage
   *
   * <p> volts Desired voltage
   */
  public void setDriveAppliedVolts(double volts);

  /**
   * Set pivot motor setpoint position
   *
   * <p> poseRevs Desired position in revs
   */
  public void setPivotSetpointPose(double poseRevs);

  /**
   * Set drive motor setpoint speed
   *
   * <p> speedMps Desired speed in Meters/Second
   */
  public void setDriveSetpointSpeed(double speedMps);

  /** Stop all module input */
  public void stop();

  /** Enable or disable module coasting for easier movement */
  public void coastingEnabled(boolean enabled);
}
