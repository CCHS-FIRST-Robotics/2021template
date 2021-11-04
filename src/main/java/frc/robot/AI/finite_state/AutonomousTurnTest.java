package frc.robot.ai.finite_state;

import frc.robot.commands.Command;
import frc.robot.state.MainState;
import frc.robot.ai.subroutines.*;

public class AutonomousTurnTest {
    double start_time = 0;
    double[][] waypoint_cloud = { { 0, 1 }, { 1, 1 }, { -1, 1 }, { -1, 0 }, { 0, -1 }, { 1, 0 }, { 0, 1 } };
    int current_target = 0;
    TurnToPoint cmd_generator;
    boolean generator_initted = false;

    public AutonomousTurnTest() {
        this.start_time = (double) System.currentTimeMillis() / 1000;
        this.current_target = 0;
        this.cmd_generator = new TurnToPoint(waypoint_cloud[this.current_target][0],
                waypoint_cloud[this.current_target][1]);
    }

    public void AutonomousInit() {
        this.current_target = 0;
        this.cmd_generator = new TurnToPoint(waypoint_cloud[this.current_target][0],
                waypoint_cloud[this.current_target][1]);
        this.generator_initted = false;
    }

    public Command getCommands(MainState main_state) {
        if (!this.generator_initted) {
            this.cmd_generator.initExit(main_state);
            this.generator_initted = true;
        }
        if (cmd_generator.exit(main_state)) {
            this.current_target++;
            if (this.current_target > this.waypoint_cloud.length - 1) {
                return new Command(0, 0);
            }
            this.cmd_generator = new TurnToPoint(waypoint_cloud[this.current_target][0],
                    waypoint_cloud[this.current_target][1]);
            this.cmd_generator.initExit(main_state);
        }
        Command output = cmd_generator.update(main_state);
        return output;
    }
}
