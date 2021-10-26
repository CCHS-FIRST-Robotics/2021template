package frc.robot.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;

public class IMUSensor extends BaseSensor {

    double ang_var = 0.0;

    public boolean log_active_sensor;
    public double log_fused_heading;

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

    public void processValue(MainState state, HardwareObjects hardware) {
        double[] xyz_dps = new double[3];
        short[] xyz_acc = new short[3];
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

        hardware.IMU.getRawGyro(xyz_dps);
        hardware.IMU.getFusedHeading(fusionStatus);

        hardware.IMU.getBiasedAccelerometer(xyz_acc);
        // 16384 = 1g

        double x_acc = (double) xyz_acc[0] * 9.81 / 32768;
        double y_acc = (double) xyz_acc[1] * 9.81 / 32768;

        double heading = fusionStatus.heading;
        heading = heading * -1 * 2 * Math.PI / 360;

        this.log_fused_heading = heading;

        double thetas = xyz_dps[3] * -1 * 2 * Math.PI / 360;

        double[] kheading = state.kalmanUpdate(state.getHeadingVal(), state.getHeadingVar(), heading, ang_var);
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