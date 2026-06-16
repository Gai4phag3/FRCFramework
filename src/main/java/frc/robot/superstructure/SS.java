package frc.robot.superstructure;

import frc.robot.ControlScheme;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

public class SS extends SubsystemBase<SS.Command> {

    public enum Flag {
        HOME,
        SCORE_LOW,
        SCORE_HIGH,
        MANUAL_UP,
        MANUAL_DOWN
    }

    public enum Command {
        IDLE,
        HOMING,
        STOWING,
        SCORING,
        MANUAL
    }

    private enum Scoring {
        RAISING,
        READY
    }

    private static final double MANUAL_VOLTS = 2.0;
    private static final double SCORE_LOW_HEIGHT_m = 1.2;

    private static SS instance;

    private final EnumSet<Flag> flags = EnumSet.noneOf(Flag.class);

    private final Elevator elevator;
    private final Drive drive;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private SS() {
        super("Superstructure");
        elevator = Elevator.getInstance();
        drive = Drive.getInstance();
        setCommand(Command.IDLE);
    }


    public void enable(Flag flag) {
        flags.add(flag);
    }

    public void disable(Flag flag) {
        flags.remove(flag);
    }

    public void set(Flag flag, boolean active) {
        if (active) {
            flags.add(flag);
        } else {
            flags.remove(flag);
        }
    }

    public void toggle(Flag flag) {
        set(flag, !has(flag));
    }

    public boolean has(Flag flag) {
        return flags.contains(flag);
    }

    @Override
    protected void inputPeriodic() {
        
    }

    @Override
    protected void handle() {
        if (has(Flag.HOME)) {
            setCommand(Command.HOMING);
            elevator.home();
        } else if (has(Flag.MANUAL_UP)) {
            setCommand(Command.MANUAL);
            elevator.manual(MANUAL_VOLTS);
        } else if (has(Flag.MANUAL_DOWN)) {
            setCommand(Command.MANUAL);
            elevator.manual(-MANUAL_VOLTS);
        } else if (has(Flag.SCORE_HIGH)) {
            setCommand(Command.SCORING);
            setSubstate(elevator.atTarget(Elevator.TOLERANCE_m) ? Scoring.READY : Scoring.RAISING);
            elevator.trackToHeight(Elevator.MAX_HEIGHT_m);
        } else if (has(Flag.SCORE_LOW)) {
            setCommand(Command.SCORING);
            setSubstate(elevator.atTarget(Elevator.TOLERANCE_m) ? Scoring.READY : Scoring.RAISING);
            elevator.trackToHeight(SCORE_LOW_HEIGHT_m);
        } else {
            setCommand(Command.STOWING);
            elevator.trackToHeight(Elevator.MIN_HEIGHT_m);
        }
    }

    @Override
    protected void outputPeriodic() {
        String[] active = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", active);
    }
}
