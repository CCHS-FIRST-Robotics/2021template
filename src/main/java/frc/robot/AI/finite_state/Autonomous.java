package frc.robot.ai.finite_state;

import frc.robot.state.*;
import frc.robot.commands.*;

public class Autonomous {
    double start_time = 0;

    public Autonomous() {
        this.start_time = (double) System.currentTimeMillis() / 1000;
    }

    public void initAuto() {
        this.start_time = (double) System.currentTimeMillis() / 1000;
    }

    public Command getCommands(MainState state) {
        if ((double) System.currentTimeMillis() / 1000 < this.start_time + 1) {
            return CommandHelper.computeCommand(0.2, 0.2);
        } else {
            return CommandHelper.computeCommand(0, 0);
        }
    }
}
