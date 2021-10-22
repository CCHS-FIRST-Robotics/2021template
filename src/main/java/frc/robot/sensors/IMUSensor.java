package frc.robot.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.HardwareObjects;
import frc.robot.state.MainState;

public class IMUSensor extends BaseSensor {

    double VARIANCE = 0.01;

    public IMUSensor(double sync_time) {
        this.VARIANCE = 0.01;
        this.SYNC_TIME = sync_time;
    }

    public boolean shouldUse(HardwareObjects hardware) {
        return (hardware.IMU.getState() == PigeonIMU.PigeonState.Ready);
    }

    public void processValue(MainState state, HardwareObjects hardware) {
        double[] xyz_dps = new double[3];
        PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

        hardware.IMU.getRawGyro(xyz_dps);
        hardware.IMU.getFusedHeading(fusionStatus);

        double heading = fusionStatus.heading;
        heading = heading * -1 * 2 * Math.PI / 360;

        double thetas = xyz_dps[3] * -1 * 2 * Math.PI / 360;

        double[] kheading = state.kalmanUpdate(state.getHeadingVal(), state.getHeadingVar(), heading, VARIANCE);
        state.setHeading(kheading[0], kheading[1]);

        double[] kangvel = state.kalmanUpdate(state.getAngVelVal(), state.getAngVelVar(), thetas,
                VARIANCE / Constants.MAIN_DT);
        state.setAngVel(kangvel[0], kangvel[1]);
    }
}