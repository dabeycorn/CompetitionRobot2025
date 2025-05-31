// Copyright (c) 2025 FRC 6423 - Ward Melville Iron Patriots
// https://github.com/FIRSTTeam6423
// 
// Open Source Software; you can modify and/or share it under the terms of
// MIT license file in the root directory of this project

package wmironpatriots.subsystems.superstructure.elevator;

public interface ElevatorIO {
  /**
   * Represents a set of measured values from elevator hardware
   *
   * @param poseMeters
   * @param setpointMeters
   * @param parentIsOk
   * @param parentPoseRevs
   * @param parentSetpointPoseRevs
   * @param parentAppliedVolts
   * @param parentStatorAmps
   * @param parentTorqueAmps
   * @param childIsOk
   * @param childPoseRevs
   * @param childSetpointPoseRevs
   * @param childAppliedVolts
   * @param childStatorAmps
   * @param childTorqueAmps
   */
  public static record LoggableState(
      double poseMeters,
      double setpointMeters,
      boolean parentIsOk,
      double parentPoseRevs,
      double parentSetpointPoseRevs,
      double parentAppliedVolts,
      double parentStatorAmps,
      double parentTorqueAmps,
      boolean childIsOk,
      double childPoseRevs,
      double childSetpointPoseRevs,
      double childAppliedVolts,
      double childStatorAmps,
      double childTorqueAmps) {}

  /**
   * @return {@link LoggableState} representing the latest hardware measurements
   */
  public LoggableState getLoggableState();

  /**
   * Sets setpoint voltage for parent and child motors
   *
   * @param volts Desired voltage
   */
  public void setMotorsAppliedVolts(double volts);

  /**
   * Set setpoint position for parent and child motors
   *
   * @param poseRevs Desired position in revs
   */
  public void setMotorsSetpointPose(double poseRevs);

  /** Stops parent and child motor */
  public void stop();

  /** Enable or disable elevator coasting for easier movement */
  public void coastingEnabled(boolean enabled);
}
