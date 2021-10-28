package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.Command;
import frc.robot.helper.SimpleMat;

public class StraightToPoint {
    double[] final_pos = { 0, 0 };

    public StraightToPoint(double target_x, double target_y) {
        this.final_pos[0] = target_x;
        this.final_pos[1] = target_y;
    }

    public Command update(MainState main_state) {
        // Compute the forward back vector factor
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.final_pos[0] - pos[0], this.final_pos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);

        double fwd_mag_fac = SimpleMat.dot(point_vec, unit_h_vec);

        return new Command(0, 0);
    }
}