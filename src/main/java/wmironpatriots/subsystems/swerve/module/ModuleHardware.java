package wmironpatriots.subsystems.swerve.module;

/** Generalized Hardware methods for a Swerve Module */
public interface ModuleHardware {
    /**
     * @return pivot motor's position in Revolutions
     */
    public double getPivotPoseRevs();

    /**
     * @return drive motor's speed in Meters Per Second
     */
    public double getDriveSpeedMps();

    /**
     * Set pivot motor's voltage setpoint
     * 
     * @param volts desired voltage setpoint
     */
    public void setPivotAppliedVolts(double volts);

    /**
     * Set drive motor's voltage setpoint
     * 
     * @param volts desired voltage setpoint
     */
    public void setDriveAppliedVolts(double volts);

    /**
     * Set pivot motor's position setpoint
     * 
     * @param volts desired position setpoint in Revolutions
     */
    public void setPivotSetpointPose(double poseRevs);

    /**
     * Set drive motor's speed setpoint
     * 
     * @param volts desired speed setpoint in MMeters Per Second
     */
    public void setDriveSetpointSpeed(double speedMps);

    /** Stop both pivot and drive motors */
    public void stop();
    
    /**
     * Toggle motor coasting
     * 
     * @param enabled should coasting be enabled?
     */
    public void coastingEnabled(boolean enabled);
}
