package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.HardwareMap.skyHardwareMap;


import java.util.Locale;

import static java.lang.Thread.sleep;





public class RevHubIMU {

    public skyHardwareMap hardware;
      private BNO055IMU imu;
      PIDController pidRotate, pidDrive;
      public double globalAngle, correction, power;
      Orientation lastAngles = new Orientation();

    // Only used or PID Forward and Rotate Methods -- 1/19/19-- Testing to see if these declarations are needed here.
    // If PIDDriveForward Works globally, then it is safe to remove these

    static final double COUNTS_PER_MOTOR_REV = 1120;        // Andmark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = 1.0;         // This is > 1.0 if motors are geared up
    static final double WHEEL_DIAMETER_INCHES = 4.0;        // For calculating circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Constructor Instantiates hardware
    public RevHubIMU(BNO055IMU imu, boolean calibrate) {
        this.imu = imu;
        if (calibrate)
            calibrate();
    }

    public void calibrate() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;

        // Set PID proportional value to start reducing power at about 50 degrees of rotation
        pidRotate = new PIDController(.005, 0,0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straght line.  P value controles how sensitive the correction is
        pidDrive = new PIDController(.05, 0,0);

        imu.initialize(parameters);
        while(!imu.isGyroCalibrated()) {
            // wait - IMU Gyro is Calibrating itself
        }

        // END of BN0555 Setup and Calibration
    }

    public double CPI () {      // helper Mehtod to return Counts Per Inch
        double CPI = COUNTS_PER_INCH;
        return CPI;
    }

    public double getAngle2() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.


        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;


        globalAngle += deltaAngle;

        lastAngles = angles;

        return (globalAngle);

    }

    // Resets the cumulative angle tracking to zero

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double[] getAngles() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES);
        double[] anglesArray = {angles.thirdAngle, angles.secondAngle, angles.firstAngle};
        return anglesArray;
    }

    public double getHeading() {
        return -getAngles()[2];
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegress(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegress(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}    // End of RevHubIMU Class