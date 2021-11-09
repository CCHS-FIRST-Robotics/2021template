package frc.robot.ai.finite_state;

import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.ai.subroutines.*;

public class AutonomousTravel {
    double start_time = 0;
    double[][] waypoint_cloud = { { 0, 3 }, { 0, 1.5 }, { 0, 0 } };
    int current_target = 0;
    StraightToPoint2 cmd_generator;
    TurnToPoint turn_generator;
    boolean generator_initted = false;
    boolean turn_state = true;

    public AutonomousTravel() {
        this.start_time = (double) System.currentTimeMillis() / 1000;
        this.current_target = 0;

        this.turn_generator = new TurnToPoint(waypoint_cloud[this.current_target][0],
                waypoint_cloud[this.current_target][1]);
        this.cmd_generator = new StraightToPoint2(waypoint_cloud[this.current_target][0],
                waypoint_cloud[this.current_target][1]);
    }

    public void AutonomousInit() {
        this.current_target = 0;
        this.turn_generator = new TurnToPoint(waypoint_cloud[this.current_target][0],
                waypoint_cloud[this.current_target][1]);
        this.cmd_generator = new StraightToPoint2(waypoint_cloud[this.current_target][0],
                waypoint_cloud[this.current_target][1]);
        this.generator_initted = false;
    }

    public Command getCommands(MainState main_state) {
        if (!this.generator_initted) {
            this.turn_generator.initExit(main_state);
            this.cmd_generator.initExit(main_state);
            this.generator_initted = true;
        }
        Command output;
        if (this.turn_state) {
            if (this.turn_generator.exit(main_state)) {
                this.turn_state = false;
                this.cmd_generator = new StraightToPoint2(waypoint_cloud[this.current_target][0],
                        waypoint_cloud[this.current_target][1]);
                this.cmd_generator.initExit(main_state);
            }
            output = this.turn_generator.update(main_state);
        } else {
            if (this.cmd_generator.exit(main_state)) {
                this.current_target++;
                this.turn_state = true;
                if (this.current_target > this.waypoint_cloud.length - 1) {
                    return new Command(0, 0);
                }
                this.turn_generator = new TurnToPoint(waypoint_cloud[this.current_target][0],
                        waypoint_cloud[this.current_target][1]);
                this.cmd_generator = new StraightToPoint2(waypoint_cloud[this.current_target][0],
                        waypoint_cloud[this.current_target][1]);
                this.turn_generator.initExit(main_state);
            }
            output = cmd_generator.update(main_state);
        }
        return output;
    }
}
