// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePos_r = 0.0;
    public double driveVel_rps = 0.0;
    public double driveVolts_V = 0.0;
    public double driveCurrent_A = 0.0;

    public boolean steerConnected = false;
    public boolean steerAbsConnected = false;
    public Rotation2d steerAbsPos_Rot2d = new Rotation2d();
    public Rotation2d steerPos_Rot2d = new Rotation2d();
    public double steerVel_rps = 0.0;
    public double steerVolts_V = 0.0;
    public double steerCurrent_A = 0.0;

    public double[] odometryTimestamps_s = new double[] {};
    public double[] odometryDrivePos_r = new double[] {};
    public Rotation2d[] odometrySteerPos_Rot2d = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  public default void stop() {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setBrake(boolean brake) {}
}
