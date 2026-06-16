package frc.robot.devices.halleffect;

import org.littletonrobotics.junction.AutoLog;

public interface HallEffectIO {

    @AutoLog
    public static class HallEffectIOInputs {
        public boolean connected = false;
        public boolean detected = false; // raw detection (post-inversion), pre-debounce
    }

    public default void updateInputs(HallEffectIOInputs inputs) {}

    /** Drives the simulated sensor state; no-op on real hardware. */
    public default void setSimDetected(boolean detected) {}
}
