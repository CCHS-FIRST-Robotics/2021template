package frc.robot;

public class Logger {
    public static void periodPrint(RobotContainer rContainer) {
        System.out.println("======Checksum: aaa_start_001========");
        // RAW Sensor data
        if (rContainer.imu_sensor.log_active_sensor) {
            System.out.println("IMU Ready");
            System.out.println("IMU Fused Heading: " + String.valueOf(rContainer.imu_sensor.log_fused_heading));
            double[] accv = rContainer.imu_sensor.log_acc;
            System.out.println("IMU Accelerometer: (" + String.valueOf(accv[0]) + ", " + String.valueOf(accv[1]) + ")");
        } else {
            System.out.println("IMU Disconected");
        }
        System.out.println("Encoder Left Rads/s: " + String.valueOf(rContainer.drive_encoder_sensor.log_l_radss));
        System.out.println("Encoder Right Rads/s: " + String.valueOf(rContainer.drive_encoder_sensor.log_r_radss));
        // State data
        System.out.println("Left Power Proportion: " + String.valueOf(rContainer.main_command.left_pwr_prop));
        System.out.println("Right Power Proportion: " + String.valueOf(rContainer.main_command.right_pwr_prop));
        System.out.println("Predicted Heading: " + String.valueOf(rContainer.main_state.getHeadingVal()));
        System.out.println("Predicted Position: (" + String.valueOf(rContainer.main_state.getPosVal()[0]) + ", "
                + String.valueOf(rContainer.main_state.getPosVal()[1]) + ")");
    }
}
