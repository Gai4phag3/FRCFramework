package frc.robot.devices.halleffect;

import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class HallEffectConfig {

    public int channel;
    public boolean inverted = false;        // if true, a raw "false" reading counts as detected
    public double debounceTime = 0.0;       // seconds; 0 disables debouncing
    public DebounceType debounceType = DebounceType.kBoth;

    public HallEffectConfig(int channel) {
        this.channel = channel;
    }

    public HallEffectConfig withInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public HallEffectConfig withDebounce(double seconds, DebounceType type) {
        this.debounceTime = seconds;
        this.debounceType = type;
        return this;
    }
}
