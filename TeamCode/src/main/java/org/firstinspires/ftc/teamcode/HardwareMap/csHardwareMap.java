package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.ReadWriteFile;


import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.UniversalConstants;
import org.firstinspires.ftc.teamcode.AutonomousData;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Vision.Detectors.GoldAlignDetector;
//import org.firstinspires.ftc.teamcode.Vision.Detectors.NavTargetDetector;

import java.io.File;
import java.util.Locale;


/* csHardwareMap.java

Purpose:

The purpose of this file is to map all physical hardware to software.  Items that get defined here include:

- Defining DC Motors
- Define Servo's
- Define IMU Parameters and Calibration Settings
- Define IC2 Sensors
- Define MISC hardware/sensors

Revision History:

7/28/19     Eric        Initial File Creation


Notes:
- This new hardware map file cleans up the old RoverRuckus HardwareMap File (removed un used items)
- You may need to included additional "import com.qualcomm.x.x" statements above
   to "pull-in" additional Java Libraries

*/

public class csHardwareMap {

    // Define Vision Detectors  (examples from RR season>
    //public GoldAlignDetector mineralDetector;
    //public NavTargetDetector navTargetDetector;


    // Define DC Motors used in Drive Train

    public DcMotor
            DriveLeftFront =null,
            DriveLeftRear =null,
            DriveRightFront = null,
            DriveRightRear = null;

    private DcMotor motor;


    // Define DC Motors used elsewhere

    public DcMotor
            LANDERHOOK = null,
            LL_DEPOSIT = null,
            PVT_EXTEND = null;



    // Define Servo's that are in use

 /*
    public Servo
        SWEEP = null,
        SWEEP_EXT = null,
        SWEEPER_EXT2 = null;

    public final double MARKER_ARM_UP = 1.0,
        MARKER_ARM_MIDDLE = 0.5,
        MARKER_ARM_DOWN = 0.0,
        PHONES_UP = .40,            //  SETS phone in UP position
        PHONES_DOWN = .77;         //  SETS phone in down position

  */


    // Define Digital I/O Limit Switches

    public DigitalChannel
        limitswBot = null,
        limitswTop = null;


    // Define Constants for Phone Position for Vision Purposes

    public final static double
            CAMERA_FORWARD_POSITION = 3.00,  // eq: Camera is 0mm in front of the robot center (Original was 3.5, we think that it is really inches)
            CAMERA_LEFT_POSITION = 7.25,     // eq. Camera is 0mm left of the robots center line
            CAMERA_VERTICAL_DISPLACEMENT = 6.0;  // Camera is 6.0 Off the field Floor

    // X-Position Pixel Value for center of robot
    public final static int ROBOT_CENTER_X = 285;


    // Local OpMode Members

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();



    // IMU Definitions
    public BNO055IMU imu;



    public void init (HardwareMap robot_AS) {
        hwMap = robot_AS;

    // Define and Initialize Motors

        DriveLeftFront = hwMap.dcMotor.get("LF");
        DriveLeftRear = hwMap.dcMotor.get("LR");
        DriveRightFront = hwMap.dcMotor.get("RF");
        DriveRightRear = hwMap.dcMotor.get("RR");

        DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        DriveLeftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors Power to Zero Power
        DriveRightFront.setPower(0);
        DriveRightRear.setPower(0);
        DriveLeftFront.setPower(0);
        DriveLeftRear.setPower(0);

    // Additional DC motor definitions for Rover Ruckus Robot
        //LANDERHOOK = hwMap.dcMotor.get("hook");
        //LL_DEPOSIT = hwMap.dcMotor.get("deposit");
        //PVT_EXTEND = hwMap.dcMotor.get("pvt");
        //SWEEP_EXT = hwMap.servo.get("sweepext");// servo
        //SWEEPER_EXT2 = hwMap.servo.get("sweepext2");

        //LANDERHOOK.setDirection(DcMotorSimple.Direction.REVERSE);
        //PVT_EXTEND.setDirection(DcMotor.Direction.FORWARD);


        //LANDERHOOK.setPower(0);
        //PVT_EXTEND.setPower(0);


        //SWEEP = hwMap.servo.get("sweep");
        //markerArm = hwMap.get(Servo.class, "markerArm");

     //Initial Digital I/O and Misc Items

        // limitswBot = hwMap.digitalChannel.cast(1);
        limitswTop = hwMap.digitalChannel.get("touch");
        limitswTop.setMode(DigitalChannel.Mode.INPUT);

        //phones = hwMap.get(Servo.class, "phones");

     //Initialize Vision Detectors
        //mineralDetector = new GoldAlignDetector(ROBOT_CENTER_X, 300, true);
        //navTargetDetector = new NavTargetDetector(hwMap, CAMERA_FORWARD_POSITION, CAMERA_LEFT_POSITION, CAMERA_VERTICAL_DISPLACEMENT);


    //------------------------------------------------------------
    // IMU - BNO055
    // Set up the parameters with which we will use our IMU.
    // + 9 degrees of freedom
    // + use of calibration file (see calibration program)
    //
    //   New Parameters File = Navigation/RevHubIMU
    //------------------------------------------------------------

        imu = hwMap.get(BNO055IMU.class, "imu");



    }
}
