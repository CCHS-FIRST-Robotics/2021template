package frc.robot.sensors;

import frc.robot.sensors.BaseSensor;
import frc.robot.state.MainState;
import frc.robot.HardwareObjects;
import com.ctre.phoenix.motorcontrol.can.*;
import java.lang.Math;

public class DriveEncoderSensor extends BaseSensor{
    double VARIANCE = 0.01;
    public DriveEncoderSensor(double sync_time){
        this.LAG_TIME = 0.0; //No Lag
        this.SYNC_TIME = sync_time;
        this.last_updated = sync_time;
    }
    public boolean shouldUse(){
        return true;
    }
    public boolean commonSense(double rpm){
        if (rpm>300){
            return false;
        }
        if (rpm<-300){
            return false;
        }
        return true;
    }
    public void processValue(MainState state){
        double l_raw = HardwareObjects.LEFT_MOTOR.getSelectedSensorVelocity(1);
        double r_raw = HardwareObjects.RIGHT_MOTOR.getSelectedSensorVelocity(1);
        //Convert to rads per sec
        double l_radss = l_raw*2*Math.PI/60;
        double r_radss = r_raw*2*Math.PI/60;
        
        double l_pred_radss = state.getLWheelVelVal();
        double r_pred_radss = state.getRWheelVelVal();

        double l_pred_var = state.getLWheelVelVar();
        double r_pred_var = state.getRWheelVelVar();

        if (commonSense(l_radss)){
            double[] l_new = state.kalmanUpdate(l_pred_radss, l_pred_var, l_radss, VARIANCE);
            state.setLWheelVel(l_new[0], l_new[1]);
        }
        if (commonSense(r_radss)){
            double[] r_new = state.kalmanUpdate(r_pred_radss, r_pred_var, r_radss, VARIANCE);
            state.setRWheelVel(r_new[0], r_new[1]);
        }

        //Logging
        System.out.println("Left Encoder Velocity: " + String.valueOf(l_radss));
        System.out.println("Right Encoder Velocity: " + String.valueOf(r_radss));
    }
}