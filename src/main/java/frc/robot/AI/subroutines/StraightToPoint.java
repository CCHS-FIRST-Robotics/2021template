package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.Command;

public class StraightToPoint {
    MainState state_ref;
    double[] final_pos = { 0, 0 };

    public StraightToPoint(double target_x, double target_y, MainState main_state) {
        this.state_ref = main_state;
        this.final_pos[0] = target_x;
        this.final_pos[1] = target_y;
    }

    public Command update() {
        return new Command(0, 0);
    }
}