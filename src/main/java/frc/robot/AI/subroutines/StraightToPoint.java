package frc.robot.ai.subroutines;

import frc.robot.state.MainState;
import frc.robot.commands.Command;
import frc.robot.helper.SimpleMat;
import java.lang.Math;

public class StraightToPoint {
    double[] final_pos = { 0, 0 };
    double startTimeSec = 0;
    double travelDistance = 0;
    int updates = 0;
    

    public StraightToPoint(double target_x, double target_y) {
        this.final_pos[0] = target_x;
        this.final_pos[1] = target_y;
    }

    public Command update(MainState main_state) {
        if (updates == 0) {
            this.startTimeSec = System.currentTimeMillis() / 1000; // Start "timer" here
            this.travelDistance = distanceToEnd(main_state); // Distance to travel overall
        }
        // Compute the forward back vector factor
        double[] pos = main_state.getPosVal();
        double[] point_vec = { this.final_pos[0] - pos[0], this.final_pos[1] - pos[1] };

        point_vec = SimpleMat.unitVec(point_vec);
        double[] unit_h_vec = SimpleMat.projectHeading(main_state.getHeadingVal(), 1);

        double fwd_mag_fac = SimpleMat.dot(point_vec, unit_h_vec);

        this.updates++;
        return new Command(0, 0);
    }

    public double timeSinceStart() {
        long currentTimeSec = System.currentTimeMillis() / 1000;
        double difference = currentTimeSec - this.startTimeSec;
        double timeDifClean = Double.parseDouble(String.format("%.2g%n", difference)); // Time in seconds to hundreths
        return timeDifClean;
    }

    public double distanceToEnd(MainState main_state) {
        double[] pos = main_state.getPosVal();
        if (this.final_pos[0] - pos[0] < 0 || this.final_pos[1] - pos[1] < 0) {
            return 0.0;
        }
        double distance = Math.sqrt(Math.pow((this.final_pos[0] - pos[0]), 2) + Math.pow((this.final_pos[1] - pos[1]), 2));
        double disClean = Double.parseDouble(String.format("%.2g%n", distance)); // Rounded to hundreths
        return disClean;
    }

    public boolean timeExit(MainState main_state) {
        // if (timeSinceStart() * [rps * wheelcircumfrence] == traveldistance) { return true }
        return false;
    }

    public boolean distanceExit(MainState main_state) {
        if (distanceToEnd(main_state) == 0) { 
            return true;
        }
        return false;
    }

    public boolean weightedEnd() { // VERY ROUGH
        // if ((timeSinceStart() * [rps * wheelcircumfrence] / traveldistance) * 10 + ((this.traveldistance - distanceToEnd) / (this.traveldistance / 10)) * 10 >= 20 ) {return true}
        return false;
    }
}