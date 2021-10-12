package frc.robot.commands;
public class Command {
    public double left_pwr_prop = 0;
    public double right_pwr_prop = 0;
    public Command(double left_pwr_prop, double right_pwr_prop){
        this.left_pwr_prop = left_pwr_prop;
        this.right_pwr_prop = right_pwr_prop;
    }
}