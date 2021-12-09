package frc.robot.map;

import frc.robot.map.LimeTarget;
import java.util.ArrayList;

public class MapBase {
    double[] start_pos = { 0, 0 };
    double start_heading = 0;
    ArrayList<LimeTarget> lime_objs = new ArrayList<LimeTarget>();

    public MapBase(int map_case) {
        switch (map_case) {
            case 1:
                this.start_pos[0] = 0;
                this.start_pos[1] = 0;
                this.start_heading = Math.PI;
                LimeTarget target_1 = new LimeTarget(1, 1, 0, Math.PI);
                this.lime_objs.add(target_1);
                break;
        }
    }
}
