package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.hardware.GyroSensor;
import java.util.ArrayList;


public class MRGyro {

    private GyroSensor gyroSensor;  // Hardware Device Object


    public MRGyro(GyroSensor gyro, boolean calibrate) {

        gyroSensor = gyro;

        if (calibrate)
            calibrate();
        zero();
    }

    public void calibrate() {
        gyroSensor.calibrate();
    }

    public void zero() {
        gyroSensor.resetZAxisIntegrator();
    }


    //Deletes all previous headings


    public int getHeading() {
        //returns heading -- adjusted for the fact that its reading backwards values
        return 360 - gyroSensor.getHeading();
    }


    public int getAngle() {
      // Returns the angle the robot has turned from the origin, negative for anythig left and positive for anything right
        return convertToAngleFromOrigin(getHeading());
    }

    private int convertToAngleFromOrigin(int heading) {
        // We convert from gyro heading to angle turned from origin alot in this class.  Thus this
        // helper method makes things eaiser to read.

        if (heading > 180) {
            return -(360 - heading);
        } else
            return heading;
    }

    public static int convertToDegrees(double radians){
        return (int) ((radians * 180) / (Math.PI));
    }

    public static double convertToRadians(int degrees) {
        return ((degrees * Math.PI) / 180);
    }
}
