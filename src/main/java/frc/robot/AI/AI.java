package frc.robot.ai;

import frc.robot.state.*;
import frc.robot.ai.finite_state.*;
import frc.robot.commands.*;

//AI IS A STATE MACHINE
public class AI {
    enum States {
        CONTROLLER, AUTONOMOUS, DISABLED
    }

    public States current_state = States.CONTROLLER;
    public Controller controller_state = new Controller();
    public Command main_command = new Command(0, 0);
    // public AutonomousTestDrive autonomousTestDrive = new AutonomousTestDrive();
    // public AutonomousTravel autonomous;
    public AutonomousTurnTest autonomous;

    public AI() {
        this.current_state = States.CONTROLLER;
        this.controller_state = new Controller();
        this.main_command = new Command(0, 0);
        this.autonomous = new AutonomousTurnTest();
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

    public String getFiniteState() {
        if (this.current_state == States.CONTROLLER) {
            return "CONTROLLER";
        } else if (this.current_state == States.AUTONOMOUS) {
            return "AUTONOMOUS";
        } else {
            return "N/A";
        }
    }

    public void setControllerState() {
        this.current_state = States.CONTROLLER;
    }

    public void setAutonomousState() {
        this.current_state = States.AUTONOMOUS;
    }
}