package frc.robot.sensors;

import frc.robot.state.MainState;

public abstract class BaseSensors{
    double lag_time;
    double last_updated;

    MainState internal_state;
    public abstract boolean shouldUse();
    public abstract boolean commonSense();
    public abstract void processValue();
}