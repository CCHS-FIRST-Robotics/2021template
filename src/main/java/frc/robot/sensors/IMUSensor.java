package frc.robot.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;
import frc.robot.helper.*;

public class IMUSensor extends BaseSensor {

    double ang_var = 0.0;

    public boolean log_active_sensor;
    public double log_fused_heading;
    public double[] log_acc = { 0, 0 };
    public double log_pitch;

    double x_acc_zero = 0;
    double[] yz_acc_zero = { 0, 9.81 };
    double yz_mag_zero = 9.81;

    public IMUSensor(double sync_time) {
        this.ang_var = Constants.BASE_HEADING_VAR;
        this.SYNC_TIME = sync_time;
    }

    public boolean shouldUse(HardwareObjects hardware) {
        this.log_active_sensor = (hardware.IMU.getState() == PigeonIMU.PigeonState.Ready);
        return (hardware.IMU.getState() == PigeonIMU.PigeonState.Ready);
    }

    void updateHeadingVar() {
        this.ang_var = this.ang_var + Constants.DELTA_VAR * Constants.MAIN_DT;
        if (this.ang_var > Constants.MAX_HEADING_VAR) {
            this.ang_var = Constants.MAX_HEADING_VAR;
        }
    }

    public void reset(HardwareObjects hardware) {
        short[] xyz_acc = new short[3];
        hardware.IMU.getBiasedAccelerometer(xyz_acc);
        double x_acc = (double) xyz_acc[0] * -9.81 / 16384;
        double y_acc = (double) xyz_acc[1] * -9.81 / 16384;
        double z_acc = (double) xyz_acc[2] * -9.81 / 16384;

        this.x_acc_zero = x_acc;
        this.yz_acc_zero[0] = y_acc;
        this.yz_acc_zero[1] = z_acc;

        this.yz_mag_zero = SimpleMat.mag(this.yz_acc_zero);

        hardware.IMU.setFusedHeading(0);
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double[] xyz_dps = new double[3];
        short[] xyz_acc = new short[3];
        double[] ypr_deg = new double[3];
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

        hardware.IMU.getRawGyro(xyz_dps);
        hardware.IMU.getFusedHeading(fusionStatus);

        hardware.IMU.getBiasedAccelerometer(xyz_acc);

        hardware.IMU.getYawPitchRoll(ypr_deg);
        // 16384 = 1g

        double x_acc = (double) xyz_acc[0] * -9.81 / 16384;
        x_acc = x_acc - x_acc_zero;

        double r_pitch = ypr_deg[1] * 2 * Math.PI / 360;
        double yt_acc = (double) xyz_acc[1] * -9.81 / 16384;
        double zt_acc = (double) xyz_acc[2] * -9.81 / 16384;
        double y_acc = yt_acc * Math.cos(r_pitch) - zt_acc * Math.sin(r_pitch);

        this.log_acc[0] = x_acc;
        this.log_acc[1] = y_acc;
        this.log_pitch = r_pitch;

        double heading = fusionStatus.heading;
        heading = heading * -1 * 2 * Math.PI / 360;
        heading = SimpleMat.angleRectifier(heading);

        this.log_fused_heading = heading;

        double thetas = xyz_dps[2] * -1 * 2 * Math.PI / 360;

        double[] kheading = state.kalmanAngleUpdate(state.getHeadingVal(), state.getHeadingVar(), heading, ang_var);
        state.setHeading(kheading[0], kheading[1]);

        double[] kangvel = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(), thetas,
                ang_var / Constants.MAIN_DT);

        state.setAngVel(kangvel[0], kangvel[1]);

        double[] kxacc = state.kalmanUpdate(state.getAccVal()[0], state.getAccVar(), x_acc, Constants.IMU_ACC_VAR);
        double[] kyacc = state.kalmanUpdate(state.getAccVal()[1], state.getAccVar(), y_acc, Constants.IMU_ACC_VAR);

        double[] new_acc = { kxacc[0], kyacc[0] };
        state.setAcc(new_acc, kxacc[1]);

        updateHeadingVar();
    }
}