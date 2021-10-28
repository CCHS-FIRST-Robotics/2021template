package frc.robot.commands;

public class Command {
    public double left_pwr_prop = 0;
    public double right_pwr_prop = 0;

    public Command(double left_pwr_prop, double right_pwr_prop) {
        this.left_pwr_prop = Math.max(Math.min(left_pwr_prop, 1), -1);
        this.right_pwr_prop = Math.max(Math.min(right_pwr_prop, 1), -1);
    }
}