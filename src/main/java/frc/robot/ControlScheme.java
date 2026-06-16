package frc.robot;

import frc.robot.Superstructure.Flag;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.util.Util;

/**
 * Maps operator input to {@link Superstructure} flags and drive input.
 *
 * <p>Replaces the old {@code ControlScheme}/{@code ElevatorControlScheme}. Each loop it feeds the
 * drivetrain stick input and sets superstructure flags from buttons — because flags are independent,
 * several can be active at once. This class only reads the controller; the superstructure decides and
 * executes.
 */
public class Controls {

    private final Superstructure ss;
    private final Drive drive;

    public Controls(Superstructure ss, Drive drive) {
        this.ss = ss;
        this.drive = drive;
    }

    /** One-time setup at teleop enable. */
    public void init() {
        drive.queueState(PathingMode.FIELD_RELATIVE);
        drive.setPathingOverride(PathingOverride.NONE);
        System.out.println("Controls initialized");
    }

    /** Reads the controller and pushes drive input + superstructure flags. Call once per loop. */
    public void update() {
        // Drivetrain stick input (ported from the old ControlScheme).
        double x_ = OI.deadband(-OI.DR.getLeftY());
        double y_ = OI.deadband(-OI.DR.getLeftX());
        double w_ = 0.5 * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        double throttle = Util.sqInput(
                1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        if (OI.DR.getPOV() == 180) {
            drive.zeroGyro();
        }
        drive.setInput(new SwerveInput(x_, y_, w_, throttle));

        // Independent superstructure flags — multiple may be active simultaneously.
        ss.set(Flag.HOME, OI.DR.getBackButton());
        ss.set(Flag.MANUAL_UP, OI.DR.getAButton());
        ss.set(Flag.MANUAL_DOWN, OI.DR.getBButton());
        ss.set(Flag.SCORE_LOW, OI.DR.getXButton());
        ss.set(Flag.SCORE_HIGH, OI.DR.getYButton());
    }
}
