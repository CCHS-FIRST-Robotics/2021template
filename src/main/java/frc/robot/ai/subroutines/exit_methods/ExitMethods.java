package frc.robot.ai.subroutines.exit_methods;

import frc.robot.Constants;
import frc.robot.state.MainState;

public class ExitMethods {
    public static double timeSinceStart(double start_time) {
        long current_time_sec = System.currentTimeMillis() / 1000;
        double difference = current_time_sec - start_time;
        double timeDifClean = Double.parseDouble(String.format("%.2g%n", difference)); // Time in seconds to hundreths
        return timeDifClean;
    }

    public static double distanceToEnd(MainState main_state, double[] final_pos) {
        double[] pos = main_state.getPosVal();
        double distance = Math.sqrt(Math.pow((final_pos[0] - pos[0]), 2) + Math.pow((final_pos[1] - pos[1]), 2));
        double disClean = Double.parseDouble(String.format("%.2g%n", distance)); // Rounded to hundreths
        return disClean;
    }

    public static double targetTime(double travel_distance) {
        return Constants.TIMER_START + travel_distance * Constants.TIMER_LEEWAY
                / ((Constants.MOTOR_MAX_RPM * 2 * Math.PI / 60) * Constants.WHEEL_RADIUS);
    }

    public static boolean timeExit(MainState main_state) {
        // if (timeSinceStart() * [rps * wheelcircumfrence] == traveldistance) { return
        // true }
        return false;
    }

    public static boolean distanceExit(MainState main_state) {
        // if (distanceToEnd(main_state) == 0) {
        // return true;
        // }
        return false;
    }

    public static boolean weightedEnd() { // VERY ROUGH
        // if ((timeSinceStart() * [rps * wheelcircumfrence] / traveldistance) * 10 +
        // ((this.traveldistance - distanceToEnd) / (this.traveldistance / 10)) * 10 >=
        // 20 ) {return true}
        return false;
    }
}
