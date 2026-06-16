package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class SwerveConfigVentura implements ISwerveConfig {
    // Indexing
    public final int FL = 0, FR = 1, BL = 2, BR = 3;

    // Measured
    public final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public final double TURN_GEAR_RATIO = 150.0 / 7.0;
    public final double MASS_KG = 65.7709;
    public final double TRACK_WIDTH_X_m = Units.inchesToMeters(18.75);
    public final double TRACK_WIDTH_Y_m = Units.inchesToMeters(18.75);
    public final double BOT_WIDTH_X_m = Units.inchesToMeters(23.75);
    public final double BOT_WIDTH_Y_m = Units.inchesToMeters(21.75);
    public final double COR_OFFSET_X_m = 0.0;
    public final double COR_OFFSET_Y_m = 0.0;
    public final double DRIVE_BASE_RADIUS_m = Math.hypot(
            TRACK_WIDTH_X_m / 2.0, TRACK_WIDTH_Y_m / 2.0); // TODO: Figure out if center of rotation is non center case

    private final double WHEEL_RADIUS_FUDGE_FACTOR = 1.0 / 1.0;
    public final double WHEEL_RADIUS_m = WHEEL_RADIUS_FUDGE_FACTOR * Units.inchesToMeters(2.0);
    public final double R2M = WHEEL_RADIUS_m * Math.PI * 2.0;

    // Odometry
    public final double ODOMETRY_FREQUENCY_Hz = 250.0;

    // Constraints
    public final double MAX_VOLTAGE_V = 13.0;
    public final double MAX_LINEAR_VEL_mps = 4.33;
    public final double MAX_LINEAR_VEL_rad_per_sec = MAX_LINEAR_VEL_mps / WHEEL_RADIUS_m;
    public final double MAX_LINEAR_VEL_CONTROLLED_mps = 4.0;
    public final double MAX_LINEAR_VEL_THROTTLED_mps = 3.5;
    public final double MAX_ANGULAR_VEL_radps = MAX_LINEAR_VEL_mps / DRIVE_BASE_RADIUS_m;
    public final double MAX_ANGULAR_VEL_THROTTLED_radps = MAX_ANGULAR_VEL_radps * 0.5;
    public final double MAX_FORWARD_ACC_mps2 = 10.0;
    public final double MAX_ANGULAR_ACC_radps2 = 2.0 * MAX_ANGULAR_VEL_radps;
    public final double MAX_TILT_XPOS_ACC_mps2 = 1000;
    public final double MAX_TILT_XNEG_ACC_mps2 = 1000;
    public final double MAX_TILT_YPOS_ACC_mps2 = 1000, MAX_TILT_YNEG_ACC_mps2 = 1000;
    public final double MAX_SKID_ACC_mps2 = 60;

    // Gains
    public final double KV_T = 1.0 / 5.5;
    public final double KP_T = 2.5;
    public final double KP_R = 3.0;

    // Current
    public final double DRIVE_STATOR_LIMIT_A = 80.0;
    public final double DRIVE_SUPPLY_LIMIT_A = 80.0;
    public final double TURN_STATOR_LIMIT_A = 80.0;
    public final double TURN_SUPPLY_LIMIT_A = 80.0;
}
