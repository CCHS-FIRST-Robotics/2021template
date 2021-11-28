import java.util.*;

import frc.robot.network.*;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.command.Command;

//add other imports

public class LimeLight {
    
    double  angle       = frc.robot.Constants.LIME_LIGHT_ANGLE;
    double  height      = frc.robot.Constants.LIME_LIGHT_HEIGHT;
    double  target1     = frc.robot.Constants.TARGET_1;
    double  target2     = frc.robot.Constants.TARGET_2;
    double  target3     = frc.robot.Constants.TARGET_3;

    //CHANGE THIS SO THAT IT USES THE PIPELINE VALUES INSTEAD
    boolean validTarget    = true;
    double  offsetH        = 1.0;   //horizontal offset
    double  offsetV        = 1.0;   //vertical offset
    double  targetArea     = 0.5;   //percentage of images taken up by target
    double  skew           = 0.0;
    double  latency        = 0.0;   //done in ms
    double  lengthH        = 10.0;  //horizontal length (in pixels)
    double  lengthV        = 10.0;  //vertical length (in pixels)
    double  dist           = 1.0;
    double  horizontal_fov = 1.0;
    double  vertical_fov   = 1.0;
    double  targetHeight   = 1.0;   //make method so that when a number is given the 
                                    //height of the target can be returned
                                    //Update: method will be in here commented and can be moved to best location

    //other variables that you can pull from pipeline include:
    // 1. tshort:  sidelength of shortest side of the fitted bounding box
    // 2. tlong:   sidelenth of longest side of the fitted bounding box
    // 3. camtran: "Results of a 3D position solution, 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)"


    //on the limelight's pixel coordinates system (0,0) is upper left -> (py,px)
    //on the limelights's normalized pixel coordinates (0,0) is the center -> (nx,ny)

    //TURN THIS INTO TWO METHODS INSTEAD OF MANY METHODS
    public double getXAngle(double lengthH, double horizontal_fov, double dist){ //figure out how to get horizontal field of view
       double nx     = (1/160) * (lengthH - 159.5); //math pulled from Additional Theory page on Limelight website
       double vpw    = Math.tan(horizontal_fov/2) * 2.0;
       double x      = vpw/2 * nx;
       double angleX = Math.atan2(1,x); //DONE WITH ROBOT FROM 1 METER AWAY

       return angleX;
    }

    public double getYAngle(double lengthV, double vertical_fov, double dist){
        double ny     = (1/120) * (119.5 - lengthV); //math pulled from Additional Theory page on Limelight website
        double vph    = Math.tan(vertical_fov/2) * 2.0;
        double y      = vph/2 * ny;
        double angleY = Math.atan2(1,y); //DONE WITH ROBOT FROM 1 METER AWAY
                                         //change 1 to distance from the target
        return angleY;
    }

    public double calculateForwardDist(double lengthV, double vertical_fov, double dist) { //INSERT PIPELINE VALUES
        double totalAngle  = getYAngle(lengthV, vertical_fov, dist) + angle;
        double heightDiff  = getTargetHeight(1) - height;

        double distance    = heightDiff/Math.tan(totalAngle);

        return distance;
    }


    //These are not really necesary because they can be calculated with the angles
    //Then to center the target you can move until the angle is zero
    //Curently these values are in "pixels" in the field of veiw - so I need to figure out how to convert them
    //(coudl use how high the limelight is and with the actual hieght in the fov - must be changed to meters though)
    public double calculateHorizontalDist(double lengthH, double horizontal_fov, double dist){ //distance from center
        double nx     = (1/160) * (lengthH - 159.5);
        double vpw    = Math.tan(horizontal_fov/2) * 2.0;
        double x      = vpw/2 * nx;
        
        return 1.0;

    }

    public double calculateVerticalDist(double lengthV, double vertical_fov, double dist) { //distance from center
        double ny     = (1/120) * (119.5 - lengthV); 
        double vph    = Math.tan(vertical_fov/2) * 2.0;
        double y      = vph/2 * ny;
        
        return 1.0;
    }

    //Finish:
    //1. horizontal dist
    //2. vertical dist
    //3. check units

    public double getTargetHeight(int targetNumber) { //distance from center

        if     (targetNumber == 1){
            return target1;
        }
        else if(targetNumber == 2){
            return target2;
        }
        else if(targetNumber == 3){
            return target3;
        }
        else {
            return 0;
        }
    }

}