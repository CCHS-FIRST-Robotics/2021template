package frc.robot.map;

import frc.robot.map.LimeTarget;
import java.util.ArrayList;
import frc.robot.Constants;

public class MapBase {
    double[] start_pos = { 0, 0 };
    double start_heading = 0;
    public ArrayList<LimeTarget> lime_objs = new ArrayList<LimeTarget>();

    public MapBase(int map_case) {
        switch (map_case) {
            case 1: //for different map arrangments or starting at different place in map
                this.start_pos[0] = 0;
                this.start_pos[1] = 0;
                this.start_heading = Math.PI;
                LimeTarget target_1 = new LimeTarget(1, 1, 0, Math.PI, frc.robot.Constants.TARGET_1);
                LimeTarget target_2 = new LimeTarget(2, 2, 0, Math.PI, frc.robot.Constants.TARGET_2);
                LimeTarget target_3 = new LimeTarget(3, 3, 0, Math.PI, frc.robot.Constants.TARGET_3);
                this.lime_objs.add(target_1);
                this.lime_objs.add(target_2);
                this.lime_objs.add(target_3);
                break;

        }
    }

}
