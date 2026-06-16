package frc.robot.devices.canrange;

import org.littletonrobotics.junction.AutoLog;

public interface CANrangeIO {

    @AutoLog
    public static class CANrangeIOInputs {
        public boolean connected = false;
        public double distance_m = 0.0;
        public boolean isDetected = false;
        public double signalStrength = 0.0;
        public double ambientSignal = 0.0;
    }

    public default void updateInputs(CANrangeIOInputs inputs) {}

    /** Drives the simulated measured distance; no-op on real hardware. */
    public default void setSimDistance(double distance_m) {}
}
