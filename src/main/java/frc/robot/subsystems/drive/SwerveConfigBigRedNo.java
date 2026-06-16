package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class SwerveConfigBigRedNo implements ISwerveConfig {

  /******************************************************
   *      Modules
   ***/
  // Measured
  public final double DRIVE_GEAR_RATIO = 6.746031746031747;

  public final double STEER_GEAR_RATIO = 21.428571428571427;
  public final double COUPLE_RATIO = 3.5714285714285716;

  // Current
  public final double DRIVE_STATOR_LIMIT_A = 150.0; // Slip current
  public final double DRIVE_SUPPLY_LIMIT_A = 60.0;
  public final double DRIVE_SUPPLY_THRES_LIMIT_A = 80.0;
  public final double DRIVE_SUPPLY_THRES_TIME_S = 2.0;
  public final double STEER_STATOR_LIMIT_A = 60.0;
  public final double STEER_SUPPLY_LIMIT_A = 80.0;

  // Gains
  public final double DRIVE_0_KP = 3.0;
  public final double DRIVE_0_KI = 0.0;
  public final double DRIVE_0_KD = 0.0;
  public final double DRIVE_0_KS = 0.0;
  public final double DRIVE_0_KV = 0.0;
  public final double DRIVE_0_KA = 0.0;

  public final double STEER_0_KP = 100;
  public final double STEER_0_KI = 0.0;
  public final double STEER_0_KD = 0; // 0.2;
  public final double STEER_0_KS = 0.0;
  public final double STEER_0_KV = 0; // 1.5;
  public final double STEER_0_KA = 0.0;

  // Pigeon
  public final int PIGEON_ID = 20;

  // FL
  public final int FL_ID_DRIVE = 1;
  public final int FL_ID_STEER = 2;
  public final int FL_ID_CANCODER = 9;
  public final double FL_ABS_OFFSET = 0.079833984375;
  public final boolean FL_INVERT_DRIVE = false;
  public final boolean FL_INVERT_STEER = false;

  // FR
  public final int FR_ID_DRIVE = 3;
  public final int FR_ID_STEER = 4;
  public final int FR_ID_CANCODER = 10;
  public final double FR_ABS_OFFSET = -0.180908203125;
  public final boolean FR_INVERT_DRIVE = true;
  public final boolean FR_INVERT_STEER = false;

  // BL
  public final int BL_ID_DRIVE = 7;
  public final int BL_ID_STEER = 8;
  public final int BL_ID_CANCODER = 12;
  public final double BL_ABS_OFFSET = -0.296630859375;
  public final boolean BL_INVERT_DRIVE = false;
  public final boolean BL_INVERT_STEER = false;

  // BR
  public final int BR_ID_DRIVE = 5;
  public final int BR_ID_STEER = 6;
  public final int BR_ID_CANCODER = 11;
  public final double BR_ABS_OFFSET = -0.3173828125;
  public final boolean BR_INVERT_DRIVE = true;
  public final boolean BR_INVERT_STEER = false;

  /******************************************************
   *       Swerve
   ***/

  public final String CAN_BUS = "canivore";

  // Measured
  public final double MASS_KG = 52.16;
  public final double TRACK_WIDTH_X_m = Units.inchesToMeters(29.5);
  public final double TRACK_WIDTH_Y_m = Units.inchesToMeters(29.5); // TODO could be wrong
  public final double BOT_WIDTH_X_m = Units.inchesToMeters(33.50);
  public final double BOT_WIDTH_Y_m = Units.inchesToMeters(33.50);
  public final double COR_OFFSET_X_m = 0.0;
  public final double COR_OFFSET_Y_m = 0.0;
  public final double DRIVE_BASE_RADIUS_m =
      Math.hypot(
          TRACK_WIDTH_X_m / 2.0,
          TRACK_WIDTH_Y_m / 2.0); // TODO: Figure out if center of rotation is non center case

  private final double WHEEL_RADIUS_FUDGE_FACTOR = 1.0 / 1.0;
  public final double WHEEL_RADIUS_m = WHEEL_RADIUS_FUDGE_FACTOR * Units.inchesToMeters(2.0);
  public final double R2M = WHEEL_RADIUS_m * Math.PI * 2.0;

  // Odometry
  public final double ODOMETRY_FREQUENCY_Hz = 250.0;

  // Constraints
  public final double MAX_VOLTAGE_V = 12.0;
  public final double MAX_LINEAR_VEL_mps = 4.0; // 4.483;
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
}