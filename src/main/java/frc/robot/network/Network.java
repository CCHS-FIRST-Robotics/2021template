package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Random;

public class Network {
    NetworkTableEntry testEntry;

    public Network() {

    }

    public void init() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable tab = inst.getTable("Test Table");
        testEntry = tab.getEntry("testEntry");
        testEntry.setDouble(1142);
    }

    public void writeNTable() {
        Random rand = new Random();
        testEntry.setDouble(rand.nextGaussian());
    }
}
