package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import java.io.File;
import java.util.Locale;

/**
 * Created by Eric on 10/20/19 - Skystone Season Hardware Mapping File
 *
 * Purpose:
 *  The purpose of this java.class file is to define all the specific hardware for
 *  the competition Robot - SkyStone 2019 Season
 *
 *  Usage:
 *
 *  Within this file, you should define mappings for:
 *      - DC Motors
 *      - Servo's
 *      - Sensors
 *      - Misc. Mapped Hardware (ICS, etc..)
 *
 *  Note:
 *      You may need to included additional "import com.qualcomm.x.x" statements above
 *      to "pull-in" additional Java Libraries
 *
 */


public class skyHardwareMap {


    public DcMotor DriveLeftFront        = null;
    public DcMotor DriveLeftRear         = null;
    public DcMotor DriveRightFront       = null;
    public DcMotor DriveRightRear        = null;
    public DcMotor LiftLeft              = null;
    public DcMotor LiftRight             = null;

    public Servo   GreenArmRight         = null;
    public Servo   PurpleArmLeft         = null;

    public ColorSensor sensorColorR, sensorColorL          = null;
    public DistanceSensor sensorDistanceR, sensorDistanceL = null;





    private DcMotor motor;


    // Rev Robotics blinkIN LED Driver.

    public RevBlinkinLedDriver blinkinLedDriver = null;
    public RevBlinkinLedDriver.BlinkinPattern pattern = null;



    // Servo Constants

    public final double MARKER_ARM_UP = 1.0;  // This was from RR Season, this most likely will not get USED.. Placeholder Only.


    // Define Digital Channel Definitions for any Limit Switches Ues

    public DigitalChannel limitswClawBracket = null;   // Defines the Limit Switch Digital Channel for the Claw Bracket
    public DigitalChannel limitswHook = null;          // Defines the Limit Switch Digita Channel for floor platform (Extra Example as of 10/20/19)



    // Constants for Phone Position for Nav Targets
    // These values are based on last years RR Robot.  Please Change Accordingly

    public final static double CAMERA_FORWARD_POSITION = 3.00;           // eg Camera is 'x' inches infront of robot center
    public final static double CAMERA_LEFT_POSITION = 7.25;             // eg Camera is 'x' inches left of robots center like
    public final static double CAMERA_VERTICAL_DISPLACEMENT = 6.0;      // eg Camera is 'x' inches off of field floor


    // X-Position pixel value for ceter of robot
    // This should be determined via calibration program

    public final static int ROBOT_CENTER_X = 285;



    // Local OpMode Members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    //REV IMU Hardware Setup

    public BNO055IMU imu;
    public boolean calibratedIMU;

    public void init (HardwareMap robot_AS) {

        hwMap = robot_AS;

        // Define and Initialize Motors


        DriveLeftFront  = hwMap.dcMotor.get("LF");
        DriveLeftRear   = hwMap.dcMotor.get("LR");
        DriveRightFront = hwMap.dcMotor.get("RF");
        DriveRightRear  = hwMap.dcMotor.get("RR");
        LiftLeft        = hwMap.dcMotor.get("LiftL");
        LiftRight       = hwMap.dcMotor.get("LiftR");

        GreenArmRight   = hwMap.servo.get("GAR");
        PurpleArmLeft   = hwMap.servo.get("PAL");

        sensorColorR    = hwMap.get(ColorSensor.class, "SCDR");  // Right colorsensor
        sensorColorL    = hwMap.get(ColorSensor.class,"SCDL");  // Left colorsensor

        sensorDistanceR = hwMap.get(DistanceSensor.class, "SCDR");  // Right colorsensor for distance
        sensorDistanceL = hwMap.get(DistanceSensor.class,"SCDL");  // Left colorsensor for distance



        DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        DriveLeftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftRight.setDirection(DcMotorSimple.Direction.FORWARD);



        // Set all motors Power to Zero Power
        DriveRightFront.setPower(0);
        DriveRightRear.setPower(0);
        DriveLeftFront.setPower(0);
        DriveLeftRear.setPower(0);

        LiftLeft.setPower(0);
        LiftRight.setPower(0);

        // Add Any additional DC Motor Definitions for SkyStone Below
        // Example:  LANDERHOOK = hwMap.dcMotor.get("hook");



        // Define any Limit Switces used for Control
        //limitswClawBracket = hwMap.digitalChannel.get("claw");  // 10/23/19  Commented out to test AS. Uncommet this out when switch is added


        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.mode = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Read the IMU configuration from the data file saved during calibration.
        // Using try/catch allows us to be specific about the error instead of
        // just showing a NullPointer exception that could come from anywhere in the program.
        calibratedIMU = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        } catch (Exception e) {
            calibratedIMU = false;
        }
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }  // End of DC Motor Reset


}

