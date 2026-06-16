package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase<Elevator.Command> {

    public enum Command {
        DISABLED,
        IDLE,
        HOMING,
        GO_TO_HEIGHT,
        MANUAL
    }

    private enum Homing {
        SEEKING,
        SETTLED
    }

    private enum Travel {
        MOVING,
        HOLDING
    }

    public static final double MIN_HEIGHT_m = 0.2191;
    public static final double MAX_HEIGHT_m = 1.724;
    public static final double STROKE_m = MAX_HEIGHT_m - MIN_HEIGHT_m;
    public static final double TOLERANCE_m = 0.04;

    private static final double MIN_HEIGHT_R = -14.812;
    private static final double MAX_HEIGHT_R = 1.262;
    private static final double METERS_TO_ROTATIONS = (MAX_HEIGHT_R - MIN_HEIGHT_R) / STROKE_m;

    private static final double HOMING_VOLTS = -0.5;

    private static Elevator instance;

    private final Motor motor;
    private final HallEffect hallEffect = new HallEffect(
            "Elevator/HallEffect",
            new HallEffectConfig(0).withInverted(true).withDebounce(0.1, DebounceType.kRising));
    private final Elevator2d measured2d = new Elevator2d("Elevator/Measured2d", new Color8Bit(200, 0, 0));
    private final Elevator2d setpoint2d = new Elevator2d("Elevator/Setpoint2d", new Color8Bit(100, 100, 100));

    private double targetHeight_m = MIN_HEIGHT_m;
    private double voltsTarget = 0.0;
    private boolean hallDetected = false;
    private boolean zeroed = false;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
            System.out.println("Elevator initialized.");
        }
        return instance;
    }

    private Elevator() {
        super("Elevator");

        MotorConfig config = new MotorConfig(15)
                .withCanbus(TunerConstants.kCANBus.getName())
                .withFollower(16, false)
                .withInverted(true)
                .withBrake(true)
                .withSupplyCurrentLimit(40.0)
                .withSensorToMechanismRatio(METERS_TO_ROTATIONS)
                .withFFGains(0.07, 0.75, 0.0, 1.0)
                .withPIDGains(18.4, 0.0, 1.5, GravityType.ELEVATOR)
                .withMotionMagic(3.1, 23.5, 150.0)
                .withSim(DCMotor.getKrakenX60Foc(2), METERS_TO_ROTATIONS, 0.1);

        motor = new Motor("Elevator/Motor", config);

        setCommand(Command.IDLE);
    }

    @Override
    protected void inputPeriodic() {
        motor.readInputs();
        hallEffect.readInputs();
        hallDetected = hallEffect.get();
    }

    @Override
    protected void handle() {
        switch (getCommand()) {
            case DISABLED:
                motor.stop();
                break;

            case IDLE:
                motor.setVoltage(0.0);
                break;

            case HOMING:
                if (firstLoop()) {
                    setSubstate(Homing.SEEKING);
                }
                if (zeroed) {
                    setSubstate(Homing.SETTLED);
                    setCommand(Command.IDLE);
                    break;
                }
                motor.setVoltage(HOMING_VOLTS);
                if (Robot.isSimulation() || hallDetected) {
                    motor.zeroPosition(MIN_HEIGHT_m);
                    zeroed = true;
                }
                break;

            case GO_TO_HEIGHT:
                setSubstate(atTarget(TOLERANCE_m) ? Travel.HOLDING : Travel.MOVING);
                motor.setMotionMagic(targetHeight_m);
                break;

            case MANUAL:
                motor.setVoltage(voltsTarget);
                break;
        }
    }

    @Override
    protected void outputPeriodic() {
        measured2d.setHeight(getHeight());
        setpoint2d.setHeight(targetHeight_m);
        measured2d.periodic();
        setpoint2d.periodic();

        Logger.recordOutput("Elevator/Height_m", getHeight());
        Logger.recordOutput("Elevator/Velocity_mps", motor.getVelocity());
        Logger.recordOutput("Elevator/TargetHeight_m", targetHeight_m);
        Logger.recordOutput("Elevator/Zeroed", isZeroed());
    }


    public void idle() {
        setCommand(Command.IDLE);
    }

    public void home() {
        setCommand(Command.HOMING);
    }

    public void trackToHeight(double height_m) {
        targetHeight_m = MathUtil.clamp(height_m, MIN_HEIGHT_m, MAX_HEIGHT_m);
        setCommand(Command.GO_TO_HEIGHT);
    }

    public void manual(double volts) {
        voltsTarget = volts;
        setCommand(Command.MANUAL);
    }

    public double getHeight() {
        return motor.getPosition();
    }

    public double getTargetHeight() {
        return targetHeight_m;
    }

    public boolean atTarget(double tol) {
        return Util.inRange(targetHeight_m - getHeight(), tol);
    }

    public boolean isZeroed() {
        return zeroed || Robot.isSimulation();
    }
}
