package frc.robot.helper;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;

/**
 * Used to keep track of general version of robot's code
 * Update Version after major changes to any part of code!
 *
 * @author Brian T.
 * @version 1.0
 */

public class Logging {
    public static String globalVersion = "0.1.0";
    public static String verDate = "10/25/2021";
    public static String loggingLastUpdate = "";

    /**
     * Returns string containing info on current stated version
     * 
     * @return String version
     */
    public String getVersion() {                                                                                                                                   
        String version = "________________________" 
        + "\n Program Version: " + globalVersion
        + "\n Version Date: " + verDate
        + "\n Last Update: " + loggingLastUpdate
        + "\n------------------------";

        return version;
    }

    /**
     * Gives a timestamp on when Logging file was last modified
     * 
     * @return String timestamp
     */
    public String logLastUpdated() {
        String filePath = System.getProperty("user.dir");

        // Gets the timestamp this file was last listed as modified
        try {

            Path file = Paths.get(filePath);
            BasicFileAttributes attr =
                Files.readAttributes(file, BasicFileAttributes.class);
            loggingLastUpdate = "" + attr.lastModifiedTime();

        } catch (IOException e) {
            e.printStackTrace();
        }

        return loggingLastUpdate;

    }

    /**
     * Gives a timestamp on when given file was last modified
     * to be called from other programs if needed
     * 
     * @return String timestamp
     */
    public String programLastUpdated(String filePath) {
        // Gets the timestamp this file was last listed as modified
        String lastUpdate = "";
        try {

            Path file = Paths.get(filePath);
            BasicFileAttributes attr =
                Files.readAttributes(file, BasicFileAttributes.class);
            lastUpdate = "" + attr.lastModifiedTime();

        } catch (IOException e) {
            e.printStackTrace();
        }

        return lastUpdate;
    }

    /**
     * For printing out information for data collection, debugging, etc.
     * to be called from other programs if needed
     * 
     * @param String[] names of info being printed
     * @param int[] respective values
     */
    public void printInfo(String[] names, int[] values) {
        divider();
        for (int i = 0; i < names.length; i++) {
            System.out.println(names[i] + ": " + values[i]);
        }
        divider();
    }

    public void divider() {
        System.out.println("===========================");
    }
    


}
