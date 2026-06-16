// Copyright 2021-2024 FRC 6328
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

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.Enums.*;
import frc.robot.util.ChassisAcceleration;
import frc.robot.util.Sharkfin;
import frc.robot.util.Util;
import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends StateMachineSubsystemBase<PathingMode> {
    // TunerConstants doesn't include these constants, so they are declared locally

    private static Drive instance;
    // private final Vision vision;

    // Constraints
    public static final double ODOMETRY_FREQUENCY_Hz = new CANBus(TunerConstants.DrivetrainConstants.CANBusName)
            .isNetworkFD()
                    ? 250.0
                    : 100.0;
    public static final double DRIVE_BASE_RADIUS_m = Units.inchesToMeters(
            Math.max(
                    Math.max(
                            Math.hypot(TunerConstants.FrontLeft.LocationX,
                                    TunerConstants.FrontLeft.LocationY),
                            Math.hypot(TunerConstants.FrontRight.LocationX,
                                    TunerConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(TunerConstants.BackLeft.LocationX,
                                    TunerConstants.BackLeft.LocationY),
                            Math.hypot(TunerConstants.BackRight.LocationX,
                                    TunerConstants.BackRight.LocationY))));

    public static final double MAX_VOLTAGE_V = 12.0;
    public static final double MAX_LINEAR_VEL_mps = 4.8; // VisionTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); //
                                                       // 4.483;
    public static final double MAX_LINEAR_VEL_CONTROLLED_mps = 4.8; // 4
    public static final double MAX_LINEAR_VEL_THROTTLED_mps = 3.5;
    public static final double MAX_ANGULAR_VEL_radps = (MAX_LINEAR_VEL_mps / DRIVE_BASE_RADIUS_m);
    public static final double MAX_ANGULAR_VEL_THROTTLED_radps = MAX_ANGULAR_VEL_radps * 0.5;
    public static final double MAX_FORWARD_ACC_mps2 = 55.0;
    public static final double MAX_AUTO_FORWARD_ACC_mps2 = 55.0;
    public static final double MAX_ANGULAR_ACC_radps2 = 22.0 * MAX_ANGULAR_VEL_radps;
    public static final double MAX_TILT_XPOS_ACC_mps2 = 100, MAX_TILT_XNEG_ACC_mps2 = 100;
    public static final double MAX_TILT_YPOS_ACC_mps2 = 100, MAX_TILT_YNEG_ACC_mps2 = 100;
    public static final double MIN_TILT_XPOS_ACC_mps2 = 20, MIN_TILT_XNEG_ACC_mps2 = 20;
    public static final double MIN_TILT_YPOS_ACC_mps2 = 20, MIN_TILT_YNEG_ACC_mps2 = 20;
    public static final double MAX_SKID_ACC_mps2 = 90;

    public double hpRotTarget = 0;



    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;

    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    public static RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1)
                            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    /*
     * public static final DriveTrainSimulationConfig mapleSimConfig =
     * DriveTrainSimulationConfig.Default()
     * .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
     * .withCustomModuleTranslations(getModuleTranslations())
     * .withGyro(COTS.ofPigeon2())
     * .withSwerveModule(new SwerveModuleSimulationConfig(
     * DCMotor.getKrakenX60(1),
     * DCMotor.getFalcon500(1),
     * TunerConstants.FrontLeft.DriveMotorGearRatio,
     * TunerConstants.FrontLeft.SteerMotorGearRatio,
     * Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
     * Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
     * Meters.of(TunerConstants.FrontLeft.WheelRadius),
     * KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
     * WHEEL_COF));
     */

    public static SwerveDriveSimulation driveSimulation = null;

    static final Lock odometryLock = new ReentrantLock();

    static {
        try {
            PP_CONFIG = RobotConfig.fromGUISettings();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
    }

    public static Drive getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    // Real robot, instantiate hardware IO implementations
                    instance = new Drive(
                            new GyroIOPigeon2(),
                            new ModuleIOTalonFX(TunerConstants.FrontLeft),
                            new ModuleIOTalonFX(TunerConstants.FrontRight),
                            new ModuleIOTalonFX(TunerConstants.BackLeft),
                            new ModuleIOTalonFX(TunerConstants.BackRight));
                    break;
                case SIM:
                    // Sim robot, TODO: instantiate physics sim IO implementations
                    instance = new Drive(
                            new GyroIO() { // TODO: IMPLEMENT sim gyro
                            },
                            new ModuleIOSim(TunerConstants.FrontLeft),
                            new ModuleIOSim(TunerConstants.FrontRight),
                            new ModuleIOSim(TunerConstants.BackLeft),
                            new ModuleIOSim(TunerConstants.BackRight));

                    break;

                default:
                    // Replayed robot, disable IO implementations
                    instance = new Drive(
                            new GyroIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            },
                            new ModuleIO() {
                            });

                    break;
            }
        }
        return instance;
    }

    private final SwerveInput si;
    private ChassisSpeeds inputSpeeds;
    private ChassisSpeeds outputSpeeds;
    private ChassisSpeeds acc;
    private ChassisSpeeds lastSpeeds;
    private ChassisSpeeds measuredAcc;
    private PathingOverride override;

    private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

    private PoseFollower poseFollower = new PoseFollower(new Pose2d(), 2.5);

    private Pose2d targetPose = new Pose2d();

    private double hpAutolock = 0;

    private double prevTx = 0;

    private boolean braked = true;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, new Pose2d());

    private Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        super("Drive");
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        System.out.println("STARTING THREAD");
        PhoenixOdometryThread.getInstance().start();

        si = new SwerveInput(SwerveInput.ZERO);
        inputSpeeds = new ChassisSpeeds();
        outputSpeeds = new ChassisSpeeds();
        acc = new ChassisSpeeds();
        lastSpeeds = new ChassisSpeeds();
        measuredAcc = new ChassisSpeeds();
        override = PathingOverride.NONE;
        gyroIO.zero();
        queueState(PathingMode.DISABLED);

        switch (Constants.currentMode) {
            case REAL:
                // vision = new Vision(
                // instance,
                // new VisionIOLimelight(VisionConstants.camera0Name, this::getRotation),
                // new VisionIOLimelight(VisionConstants.camera1Name, this::getRotation));
                break;
            case SIM:
                // vision = new Vision(
                // this,
                // new VisionIOPhotonVisionSim(
                // camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                // new VisionIOPhotonVisionSim(
                // camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));
                break;
            default:
                // vision = new Vision(this, new VisionIO() {
                // }, new VisionIO() {
                // });
                break;
        }
    }

    @Override
    public void inputPeriodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (var module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();

        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.inputPeriodic();
        }

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                double tempscale = Units.inchesToMeters(1); // Fixes sim twist calc
                moduleDeltas[0].distanceMeters *= tempscale;
                moduleDeltas[1].distanceMeters *= tempscale;
                moduleDeltas[2].distanceMeters *= tempscale;
                moduleDeltas[3].distanceMeters *= tempscale;
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case DISABLED:
                if (stateInit()) { // First time init stuff per entry of state
                }
                inputSpeeds = new ChassisSpeeds();
                break;
            case FIELD_RELATIVE:
                // System.out.println("field relative");
                double maxLinearVel_mps = Util.lerp(MAX_LINEAR_VEL_THROTTLED_mps, MAX_LINEAR_VEL_mps,
                        si.throttle);
                double maxAngularVel_radps = Util.lerp(MAX_ANGULAR_VEL_THROTTLED_radps, MAX_ANGULAR_VEL_radps,
                        si.throttle);

                // Circular input processing
                double inputMagnitude = Util.sqInput(Math.hypot(si.xi, si.yi));

                double x_ = si.xi * maxLinearVel_mps;
                double y_ = si.yi * maxLinearVel_mps;
                double w_ = si.wi * maxAngularVel_radps;

                if (inputMagnitude > 1.0) {
                    x_ = x_ / inputMagnitude;
                    y_ = y_ / inputMagnitude;
                }

                inputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_, y_, w_, getRotation());

                switch (override) {
                    case TRACKING:
                        break;
                    case BASELOCK:
                        break;
                    case INTAKING:
                        break;
                    case NONE:
                        break;
                    case PROCESSOR_PULLBACK:
                        break;
                    case PULLBACK:
                        break;
                    case HP:
                        break;

                    default:
                        break;
                }

                break;
            case POSE_FOLLOWING:
                inputSpeeds = poseFollower.process();
                break;
            case PATH_FOLLOWING:
                break;
            case AUTO_TRACKING:
                break;

            case HP_INTAKE:
                break;
            default:

        }

        ChassisSpeeds measuredSpeeds = getChassisSpeeds();
        measuredAcc = ChassisAcceleration.calculate(lastSpeeds, measuredSpeeds, Constants.globalDelta_s);
        lastSpeeds = measuredSpeeds;
        ChassisSpeeds inputAcc = ChassisAcceleration.fromChassisSpeeds(measuredSpeeds, inputSpeeds,
                Constants.globalDelta_s);

        acc = inputAcc;

        acc = accLimitForward(acc, measuredSpeeds);
        acc = accLimitAngular(acc, measuredSpeeds);
        acc = accLimitTilt(acc);
        acc = accLimitSkid(acc);

        outputSpeeds = ChassisAcceleration.fromAcceleration(measuredSpeeds, acc, Constants.globalDelta_s);

        // Antiskew
        double px = -outputSpeeds.vyMetersPerSecond;
        double py = outputSpeeds.vxMetersPerSecond;

        double kSkew = -0.002;

        outputSpeeds.vxMetersPerSecond += px * outputSpeeds.omegaRadiansPerSecond * kSkew;
        outputSpeeds.vyMetersPerSecond += py * outputSpeeds.omegaRadiansPerSecond * kSkew;

        // Log speeds and accelerations
        Logger.recordOutput("Drive/Speeds/Input", new ChassisSpeeds(si.xi, si.yi, si.wi));
        Logger.recordOutput("Drive/Speeds/InputVel", inputSpeeds);
        Logger.recordOutput("Drive/Speeds/MeasuredVel", measuredSpeeds);
        Logger.recordOutput("Drive/Speeds/OutputVel", outputSpeeds);
        Logger.recordOutput("Drive/Speeds/InputAcc", inputAcc);
        Logger.recordOutput("Drive/Speeds/MeasuredAcc", measuredAcc);
        Logger.recordOutput("Drive/Speeds/OutputAcc", acc);
    }

    public boolean inBounds(double tz, double tx) {
        double k = 1;
        return tx < tz * k && tz > -tz * k;
    }


    @Override
    public void outputPeriodic() {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = outputSpeeds; // ChassisSpeeds.discretize(outputSpeeds, Constants.globalDelta_s);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds/* , CENTER_OF_ROT */);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_VEL_mps);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        double maxModuleVel = 0.0;
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
            if (Math.abs(optimizedSetpointStates[i].speedMetersPerSecond) > MAX_LINEAR_VEL_CONTROLLED_mps) {
                maxModuleVel = Math.abs(optimizedSetpointStates[i].speedMetersPerSecond);
            }
        }

        // Determine if openloop control should be used
        Module.Mode mode = (maxModuleVel > MAX_LINEAR_VEL_CONTROLLED_mps) ? Module.Mode.HIGH_SPEED
                : Module.Mode.HIGH_CONTROL;

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        } else {
            for (var module : modules) {
                module.outputPeriodic(mode);
            }
        }

        // Log setpoint states
        Logger.recordOutput("Drive/Modules/Mode", mode);
        Logger.recordOutput("Drive/Modules/Setpoints", setpointStates);
        Logger.recordOutput("Drive/Modules/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/PathingOverride", override.name());
        Logger.recordOutput("Drive/TargetPose", targetPose);
        Logger.recordOutput("Drive/GyroAngle", getRotation().getDegrees());
    }

    public boolean velUnder(double mag) {
        double measured = Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);

        return measured < mag;
    }


    public void setPathingOverride(PathingOverride override) {
        this.override = override;
    }

    public void setInput(SwerveInput i) {
        si.set(i);
    }

    public void setAutoSpeeds(ChassisSpeeds speeds) {
        this.autoSpeeds = speeds;
    }

    public void setMaxFollowerSpeed(double maxSpeed) {
        this.poseFollower = new PoseFollower(targetPose, maxSpeed);
    }

    public void setHPAutolock(double autolock) {
        this.hpAutolock = autolock;
    }

    public void toggleBrake() {
        for (var module : modules) {
            module.setBrakeMode(!braked);
        }
        braked = !braked;
    }

    public void zeroGyro() {
        gyroIO.zero();
    }

    public ChassisSpeeds accLimitForward(ChassisSpeeds acc, ChassisSpeeds vel) {
        ChassisSpeeds res = acc;
        double accMag_mps2 = ChassisAcceleration.magnitude(acc);
        double velMag_mps = ChassisAcceleration.magnitude(vel);
        double accAng_rad = ChassisAcceleration.angle(acc);
        double velAng_rad = ChassisAcceleration.angle(vel);

        if (accMag_mps2 == 0.0) {
            return res;
        }

        if (velMag_mps == 0.0) {
            res.vxMetersPerSecond *= MAX_FORWARD_ACC_mps2 / accMag_mps2;
            res.vyMetersPerSecond *= MAX_FORWARD_ACC_mps2 / accMag_mps2;
        } else {

            double alpha = Math.cos(accAng_rad - velAng_rad);

            if (alpha > 0) {
                double max = DriverStation.isAutonomous() ? MAX_AUTO_FORWARD_ACC_mps2 : MAX_FORWARD_ACC_mps2;
                double maxFwdAcc = max * (1.0 - velMag_mps / MAX_LINEAR_VEL_mps);
                double outMag = Math.min(accMag_mps2, maxFwdAcc);
                res.vxMetersPerSecond *= outMag / accMag_mps2;
                res.vyMetersPerSecond *= outMag / accMag_mps2;
            }
        }

        return res;

    }

    public ChassisSpeeds accLimitAngular(ChassisSpeeds acc, ChassisSpeeds vel) {
        ChassisSpeeds res = acc;

        double maxAngAcc = MAX_ANGULAR_ACC_radps2 * (1.0 - Math.abs(vel.omegaRadiansPerSecond / MAX_ANGULAR_VEL_radps));

        double angularAcc_radps2 = Math.copySign(
                Math.min(maxAngAcc, Math.abs(acc.omegaRadiansPerSecond)), acc.omegaRadiansPerSecond);
        res.omegaRadiansPerSecond = angularAcc_radps2;

        return res;
    }

    public ChassisSpeeds accLimitTilt(ChassisSpeeds in) {
        ChassisSpeeds res = in;
        double target_bias = 0.5;
        double h = (1.0 - target_bias) * Elevator.getInstance().getHeight()
                + target_bias * Elevator.getInstance().getTargetHeight();

        double alpha = Util.unlerp(Elevator.MAX_HEIGHT_m, Elevator.MIN_HEIGHT_m, h);

        double maxXPosAcc = Util.lerp(MIN_TILT_XPOS_ACC_mps2, MAX_TILT_XPOS_ACC_mps2, alpha);
        double maxXNegAcc = Util.lerp(MIN_TILT_XNEG_ACC_mps2, MAX_TILT_XNEG_ACC_mps2, alpha);
        double maxYPosAcc = Util.lerp(MIN_TILT_YPOS_ACC_mps2, MAX_TILT_YPOS_ACC_mps2, alpha);
        double maxYNegAcc = Util.lerp(MIN_TILT_YNEG_ACC_mps2, MAX_TILT_YNEG_ACC_mps2, alpha);

        res.vxMetersPerSecond = Util.limit(in.vxMetersPerSecond, -maxXNegAcc,
                maxXPosAcc);
        res.vyMetersPerSecond = Util.limit(in.vyMetersPerSecond, -maxYNegAcc,
                maxYPosAcc);

        return res;
    }

    public ChassisSpeeds accLimitSkid(ChassisSpeeds acc) {
        ChassisSpeeds res = acc;
        double accMag_mps2 = ChassisAcceleration.magnitude(acc);

        double outMag = Math.min(accMag_mps2, MAX_SKID_ACC_mps2);
        if (accMag_mps2 != 0.0) {
            res = acc.times(outMag / accMag_mps2);
        }

        return res;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards forwards) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Stops the drive. */
    public void stop() {
        setInput(new SwerveInput(0, 0, 0));
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    public void updateSimulationField() {
        SimulatedArena.getInstance().simulationPeriodic();

        Logger.recordOutput("Sim/Pose", driveSimulation.getSimulatedDriveTrainPose());
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getMeasuredChassisAcceleration() {
        return measuredAcc;
    }

    public ChassisSpeeds getNextChassisAcceleration() {
        return acc;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    // @Override
    // public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    //     poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // }

    public void setTargetPose(Pose2d tPose) {
        setTargetPose(tPose, MAX_LINEAR_VEL_mps);
    }

    public void setTargetPose(Pose2d tPose, double maxVel) {
        this.targetPose = tPose;
        this.poseFollower.setParams(tPose, maxVel);
    }

    public void setTargetPose(Pose2d tPose, double maxVel, double translate_kp, double rotate_kP) {
        this.targetPose = tPose;
        this.poseFollower.setParams(tPose, maxVel, translate_kp, rotate_kP);
    }

    public PathingOverride getOverride(){
        return override;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        double i2m = Units.inchesToMeters(1.0);
        return new Translation2d[] {
                new Translation2d(i2m * TunerConstants.FrontLeft.LocationX,
                        i2m * TunerConstants.FrontLeft.LocationY),
                new Translation2d(i2m * TunerConstants.FrontRight.LocationX,
                        i2m * TunerConstants.FrontRight.LocationY),
                new Translation2d(i2m * TunerConstants.BackLeft.LocationX,
                        i2m * TunerConstants.BackLeft.LocationY),
                new Translation2d(i2m * TunerConstants.BackRight.LocationX,
                        i2m * TunerConstants.BackRight.LocationY)
        };
    }
}