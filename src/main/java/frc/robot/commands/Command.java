package frc.robot.commands;

public class Command {
    public double left_pwr_prop = 0;
    public double right_pwr_prop = 0;

    public Command(double left_pwr_prop, double right_pwr_prop) {
        this.left_pwr_prop = Math.max(Math.min(left_pwr_prop, 1), -1);
        this.right_pwr_prop = Math.max(Math.min(right_pwr_prop, 1), -1);
    }

    public void diffDrive(double fwd_cmd, double turn_cmd) {
        fwd_cmd = Math.max(Math.min(fwd_cmd, 1), -1);
        turn_cmd = Math.max(Math.min(turn_cmd, 1), -1);
        double temp_left = fwd_cmd - turn_cmd;
        double temp_right = fwd_cmd + turn_cmd;
        double max_cmd = Math.max(Math.abs(temp_left), Math.abs(temp_right));
        if (max_cmd > 1) {
            temp_left = temp_left / max_cmd;
            temp_right = temp_right / max_cmd;
        }
        this.left_pwr_prop = temp_left;
        this.right_pwr_prop = temp_right;
    }
}