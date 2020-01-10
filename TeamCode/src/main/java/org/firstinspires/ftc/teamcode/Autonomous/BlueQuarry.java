package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.HardwareMap.skyHardwareMap;

import android.app.Activity;
import android.view.View;

import java.util.Locale;

/* 10/29/19 - Good Lesson Learned here....
 *
 *   "Even the seasoned Programmer/Engineer can sometimes forget the basics....."
 *
 *  Problem:  When we executed this code, it threw an error indicating that we needed to
 *            setTargetPosition() before engaging the setMode(DcMotor.RunMode.RUN_TO_POSITION)
 *
 *  Troubleshooting:
 *     We attempted many things, including:
 *           1. Complete reconfigure of the RC Configuration file
 *           2. manually adding the setTargetPosition(0) within the method
 *           3. Manipulate a known working PIDDrieForward() method used in the 2018 State Comps
 *
 *  Resolution:
 *       Caleb to the rescue!   He discovered the following code segment was wrong, corrected it and
 *       all the errors went away, robot exeuted the commands without issue and worked perfectly.
 *
 *  Wrong Command:
 *       @Autonomous(name="Blue Build, group=AutonomousData.OFFICIAL_GROUP)
 *
 *  Corrected Command:
 *       @Autonomous(name="Blue Build", group = "CS9977-test")
 *
 *  Summary:
 *       The group= " " statement should be named the same as the Robot Controller Config Name on the phone.
 *
 *        The AutonomousData.OFFICIAL_GROUP was ported from the 2018 season, and was not properly functioning.   We will delete it
 *        and or correct that file contents to match the 2019-20 SkyStone Season parameters.
 *
 *
 */


@Autonomous(name="Blue Quarry", group = "calebs_robot")

public class BlueQuarry extends LinearOpMode {


    // Decalre hardware

    skyHardwareMap robot2 = new skyHardwareMap();
    ElapsedTime runtime = new ElapsedTime();


    // Created Rev Robotics BlinkIN instances

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    // REV Robotics Color Sensor + Distance Sensor Rev v3 Definitions
    boolean capstoneDetected = false;



    //Create IMU Instance
    int A;
    int capCount = 0;
    PIDController pidDrive, pidRotate;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    //Define Drivetrain Variabeles

    static final double COUNTS_PER_MOTOR_REV = 753.2;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = .69;    // This is > 1.0 if motors are geared up ____  Using OVerdrive gearing with Pico Uno boxes  40 gear to 35 gear over-drive
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        robot2.init(hardwareMap);
        setupIMU();

        // Configuration setup for Capstone Detection using REV Color/Distance Sensor v3

        // wait for the start button to be pressed.

        robot2.GreenArmRight.setPosition(.99);

        // This chunk of code gets around the Motorola E4 Disconnect bug.  Should be fixed in SDK 5.3, but adding it as a "backup - JUST IN CASE!!!"
        //
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        waitForStart();


        // Set the Initial; Blinkin Color Schema to Aqua (Represents CS9977 Team Colors
        //robot2.blinkinLedDriver.setPattern(pattern);
        //robot2.pattern = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE;

        /* Use this section to write the steps for Autonomous Movements
         *
         *  things we should include here are:
         *      1. Motor Movements
         *      2. Vision Detection via TensorFlow, VuForia, DodgeCV (depending on what we decide to use)
         *      3. Sensor input/output  (Color Sensors, Distance Sensors, Limit Switches, etc..)
         *      4. Navigation Utilities (as needed)
         */


        //  At this point of the testing, the robot should be back against the wall.   Any margin of error, means we need to tune the PID values
        //
        // 10/29/19  -   Above PID drive functions worked, and should be good for Merging into the Competition Software Release

        // Setup Timer for As mode


        // added the LED Signal Codes 12/12/19
        // Need to Test
        while (runtime.seconds() > 30) {       // Set LED pattern to GREE; Indicates that were starting/within 30-10 seconds of AS mode
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern); // Set the LED pattern to GREEN


            if (runtime.seconds() < 10) {      // Set LED patterbn to RED;  Indicsates that we are within the last 10 seconds of AS mode.
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(pattern);  // SET LED pattern to RED
            }
        }

        // setup PID Drive Commands to get to first Capstone closest to bridge and align robot with color sensor on first Capstone

        PIDDrivebackward(.5, 90, 26);
       // PIDDriveStrafeRight(.5,90,10);

        // Run a test to see if first block is captstone




        while (opModeIsActive()) {
            if (capDetect() == true && robot2.sensorDistanceR.getDistance(DistanceUnit.INCH) <= 2.0) {
                telemetry.addLine("Skystone is Found");
                telemetry.addData("Distance (inches) is: ", String.format(Locale.US, "%.02f", robot2.sensorDistanceR.getDistance(DistanceUnit.INCH)));
                telemetry.addData("Red ", robot2.sensorColorR.red());
                telemetry.addData("Green ", robot2.sensorColorR.green());
                telemetry.addData("Blue ", robot2.sensorColorR.blue());
                telemetry.update();
                //PIDDriveStrafeRight(1,90,1);
                PIDDrivebackward(1,90,3);
                robot2.GreenArmRight.setPosition(0);   // Lower Servo arm to grab Skystone
                sleep(1000);
                break;

            } else if (capCount >= 6  ){
                break;
            }

                else {
                telemetry.addLine("Skystone not found");


                     capCount = capCount + 1;

            // Robot should now be infront of First capstone.  Invoke the Detection Method

                if (capCount == 1) {
                    RotateLeft(1,2);
                }if (capCount == 4){
                    RotateRight(1,2);
                }

                PIDDriveStrafeRight(.5,90,10);

              sleep(500);
            }

        }

        PIDDriveForward(.5,90,5);



    }

    // This section is a placeholder for Vision Detection of the Skystone
    //  Methods we can choose from include:
    //     1. Using REV3 Color/Distance Sensor
    //     2. Using TensorFlor Vision
    //     3. Using OpenCV Vision



    // This second is a placeholder for Moving the Foundation to the build zone corner
    //  Place frame work here



    // This is the last placeholder for returning to mid-field to gain the parking points.
    // Pace framework here.




// Ends runOpMode..... END OF Autonomous Main Program.....


//     THIS SECTION BEGINS ALL OF THE PROGRAMS METHOD(S)
//


    public void setupIMU() {
        // Initalize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        pidRotate = new PIDController(.005, 0, 0);            // Kp was 0  1/24/19

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(0.02, 0, 0);             // Kp was .02  1/24/19

        telemetry.addData("Mode", "calibrating IMU.....");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }  // End of setupIMU() Method

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

    }   // End of getAngle2() Method

    /**
     * Resets the cumulative angle tracking to zero.
     */

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;

    }   // Endof ResetAngle() Method


    //  The following Methods are PIDDrivexxxx Definitions.  These are the main methods called to navigate the play Field

    public void PIDDriveForward(double speed, double angle, int distance) {    // Added: 1/18/19

        // This is an attempt to use PID only, no encoders


        /* Things to pass the Method
         *
         * 1. speed
         * 2. Angle
         * 3. distance
         *
         * Example:  PIDDriveForward(.50,90, 24);        // Drive robot forward in a straight line for 4" @ 50% Power
         */

        // Setup Straight Line Driving


        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(getAngle2());


        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior


        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        /* Setup Default Robot Position...  setTargetPosition= null
         *  This section was added after our initial attempts to re-use these
         *  PID controls failed.   Error message on the DS phone indicated
         *  we needed to setTargetPostion before running to position
         *
         *  I am adding this is a test, to see if we initialize the defauly
         *  position to 0 (robot against the wall), knowing that we will re-calculate the positon
         *  later in this method.
         *
         *  For competition, we will need to be more accurate, most likely.
         */

        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        telemetry.addLine("Just setTarget Position");
        telemetry.update();



        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Set RUN_TO_POS");
        telemetry.update();
        sleep(500);

        // Stop Motors and set Motor Power to 0
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);

        double InchesMoving = (distance * COUNTS_PER_INCH);


        // Set Target
        robot2.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy()
                && robot2.DriveLeftFront.isBusy() && robot2.DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            robot2.DriveRightFront.setPower(speed + correction);
            robot2.DriveLeftFront.setPower(speed + correction);
            robot2.DriveRightRear.setPower(speed + correction);
            robot2.DriveLeftRear.setPower(speed + correction);
        }    // This brace closes out the while loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //PIDstopALL()
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);

    }   // END OF PIDDriveForward

    public void PIDDrivebackward(double speed, double angle, int distance) {    // Added: 1/18/19

        // This is an attempt to use PID only, no encoders


        /* Things to pass the Method
         *
         * 1. speed
         * 2. Angle
         * 3. distance
         *
         * Example:  PIDDriveForward(.50,90, 24);        // Drive robot forward in a straight line for 4" @ 50% Power
         */

        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();

        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(getAngle2());


        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior


        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);


        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL()
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);


        // Set Target
        robot2.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) -InchesMoving);


        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy()
                && robot2.DriveLeftFront.isBusy() && robot2.DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            robot2.DriveRightFront.setPower(speed + correction);
            robot2.DriveLeftFront.setPower(speed + correction);
            robot2.DriveRightRear.setPower(speed);
            robot2.DriveLeftRear.setPower(speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //PIDstopALL()
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);

    }   // END OF PIDDriveBackward

    public void PIDDrivebackward2(double speed, double angle, int distance) {    // Added: 1/18/19

        // This is an attempt to use PID only, no encoders


        /* Things to pass the Method
         *
         * 1. speed
         * 2. Angle
         * 3. distance
         *
         * Example:  PIDDriveForward(.50,90, 24);        // Drive robot forward in a straight line for 4" @ 50% Power
         */

        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();

        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(getAngle2());


        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior


        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);


        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL()
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);


        // Set Target
        robot2.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) -InchesMoving);


        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy()
                && robot2.DriveLeftFront.isBusy() && robot2.DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            robot2.DriveRightFront.setPower(speed + correction);
            robot2.DriveLeftFront.setPower(speed + correction);
            robot2.DriveRightRear.setPower(speed);
            robot2.DriveLeftRear.setPower(speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //PIDstopALL()
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);

    }   // END OF PIDDriveBackward


    public void PIDDriveStrafeRight(double speed, double angle, int distance) {    // Added: 1/18/19

        // This is an attempt to use PID only, no encoders


        /* Things to pass the Method
         *
         * 1. speed
         * 2. Angle
         * 3. distance
         *
         * Example:  PIDDriveForward(.50,90, 24);        // Drive robot forward in a straight line for 4" @ 50% Power
         */

        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();

        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(getAngle2());


        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior


        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);


        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL();
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);


        // Set Target
        robot2.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) -InchesMoving);


        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy()
                && robot2.DriveLeftFront.isBusy() && robot2.DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            robot2.DriveRightFront.setPower(speed + correction);
            robot2.DriveLeftFront.setPower(speed + correction);
            robot2.DriveRightRear.setPower(speed + correction);
            robot2.DriveLeftRear.setPower(speed + correction);
        }    // This brace closes out the while loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL();
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);


    }   // END OF PIDDriveStrafeRight

    public void PIDDriveStrafeLeft(double speed, double angle, int distance) {    // Added: 1/18/19

        // This is an attempt to use PID only, no encoders


        /* Things to pass the Method
         *
         * 1. speed
         * 2. Angle
         * 3. distance
         *
         * Example:  PIDDriveForward(.50,90, 24);        // Drive robot forward in a straight line for 4" @ 50% Power
         */

        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, 0);
        pidDrive.enable();

        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(getAngle2());


        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior


        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);


        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL();
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);


        // Set Target
        robot2.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy()
                && robot2.DriveLeftFront.isBusy() && robot2.DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            robot2.DriveRightFront.setPower(speed + correction);
            robot2.DriveLeftFront.setPower(speed);
            robot2.DriveRightRear.setPower(speed);
            robot2.DriveLeftRear.setPower(speed + correction);
        }    // This brace closes out the while loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL();
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);

    }  // END of PIDDriveStrafeLeft


    public void RotateLeft(double speed, int distance) {
        // Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);


        // Set RUN_TO_POSITION

        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot2.DriveRightFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to Rotate Left
        robot2.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) -InchesMoving);

        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy() && robot2.DriveLeftRear.isBusy() && robot2.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot2.DriveRightFront.setPower(MoveSpeed);
            robot2.DriveRightRear.setPower(MoveSpeed);
            robot2.DriveLeftFront.setPower(MoveSpeed);
            robot2.DriveLeftRear.setPower(MoveSpeed);

        }  // THis brace close out the while Loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot2.DriveRightFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveLeftRear.setPower(0);

    } // This brace closes out RotateLeft

    public void RotateRight(double speed, int distance) {
        // Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);


        // Set RUN_TO_POSITION

        robot2.DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Motor Power to 0
        robot2.DriveRightFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveLeftRear.setPower(0);


        double InchesMoving = (distance * COUNTS_PER_INCH);

        // Set Target to RotateRight
        robot2.DriveRightFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) InchesMoving);

        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy() && robot2.DriveLeftRear.isBusy() && robot2.DriveLeftFront.isBusy()) {

            // wait for robot to move to RUN_TO_POSITION setting

            double MoveSpeed = speed;

            // Set Motor Power
            robot2.DriveRightFront.setPower(MoveSpeed);
            robot2.DriveRightRear.setPower(MoveSpeed);
            robot2.DriveLeftFront.setPower(MoveSpeed);
            robot2.DriveLeftRear.setPower(MoveSpeed);

        }  // THis brace close out the while Loop

        //Reset Encoders
        robot2.DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot2.DriveRightFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveLeftRear.setPower(0);
    }  // This brace closes out RotateRight Method




    private void setZeroPowerBrakes() {
        //Initialize Mecanum Wheel DC Motor Behavior
        robot2.DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot2.DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot2.DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot2.DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public boolean capDetect (boolean isFound) {

        //boolean isFound;

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        int RevCondition = 3;      // 3 = Using Rev Color Sensor V3;  2 = Using Rev Color Sensor V2(older)
        int ColorCondition = 0;    // Initialize ColorCondition variable to zero

        //  END OF Capston Configuration setup parameters

        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (robot2.sensorColorR.red() * SCALE_FACTOR),
                (int) (robot2.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot2.sensorColorR.blue() * SCALE_FACTOR),
                hsvValues);


        //  This is the algorithm to Normailze the Blue channel.
    /*
      Concept:
         Normailize the Blue channel.    Multiple the Red * Gree deviced by Blue^2
         If the Color Condition <= RevCondition # (3); then we are seeing the capstone (black)
          otherwise we are seeing soemthing other than black, and not seeing the capston
          Courtesy of FTC 5898 Youtube Explaination.  (https://www.youtube.com/watch?v=i0AskHFkZ94)
     */
        ColorCondition = (robot2.sensorColorR.red() / robot2.sensorColorR.blue() * (robot2.sensorColorR.green() / robot2.sensorColorR.blue()));

        if(ColorCondition <= RevCondition) {
            telemetry.addLine("capstone detected");
            isFound = true;
            return isFound;
        } else
            telemetry.addLine("Capstone Not Detected");
        isFound = false;
        return isFound;
    }

    public boolean capDetect () {
        //boolean isFound;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        int RevCondition = 3;      // 3 = Using Rev Color Sensor V3;  2 = Using Rev Color Sensor V2(older)
        int ColorCondition = 0;    // Initialize ColorCondition variable to zero
        //  END OF Capston Configuration setup parameters
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (robot2.sensorColorR.red() * SCALE_FACTOR),
                (int) (robot2.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot2.sensorColorR.blue() * SCALE_FACTOR),
                hsvValues);
        //  This is the algorithm to Normailze the Blue channel.
    /*
      Concept:
         Normailize the Blue channel.    Multiple the Red * Gree deviced by Blue^2
         If the Color Condition <= RevCondition # (3); then we are seeing the capstone (black)
          otherwise we are seeing soemthing other than black, and not seeing the capston
          Courtesy of FTC 5898 Youtube Explaination.  (https://www.youtube.com/watch?v=i0AskHFkZ94)
     */
        ColorCondition = (robot2.sensorColorR.red() / robot2.sensorColorR.blue() * (robot2.sensorColorR.green() / robot2.sensorColorR.blue()));
        if(ColorCondition <= RevCondition) {
            telemetry.addData("Color Condition equals: ", ColorCondition);
            return true;
        } else
        return false;
    }


}