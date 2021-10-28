package frc.robot.ai;

import frc.robot.state.*;
import frc.robot.ai.finite_state.*;
import frc.robot.commands.*;

//AI IS A STATE MACHINE
public class AI {
    enum States {
        CONTROLLER, AUTONOMOUS
    }

    public States current_state = States.CONTROLLER;
    public Controller controller_state = new Controller();
    public Command main_command = new Command(0, 0);
    public Autonomous autonomous = new Autonomous();

    public AI() {
        this.current_state = States.CONTROLLER;
        this.controller_state = new Controller();
        this.main_command = new Command(0, 0);
    }

    public Command getCommand(MainState state) {
        switch (this.current_state) {
            case CONTROLLER:
                main_command = this.controller_state.getCommands(state);
                break;
            case AUTONOMOUS:
                main_command = this.autonomous.getCommands(state);
                break;
        }
        return main_command;
    }

    public void setControllerState() {
        this.current_state = States.CONTROLLER;
    }

    public void setAutonomousState() {
        this.current_state = States.AUTONOMOUS;
    }
}