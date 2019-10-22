package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.HardwareMap.skyHardwareMap;

//


@Autonomous(name="Red Build", group = AutonomousData.OFFICIAL_GORUP)

public class RedBuild extends LinearOpMode {


    // Decalre hardware

    skyHardwareMap robot2 = new skyHardwareMap();
    ElapsedTime runtime = new ElapsedTime();

    // Created Rev Robotics BlinkIN instances

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;



    //Create IMU Instance
    PIDController pidDrive, pidRotate;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    //Define Drivetrain Variabeles

    static final double COUNTS_PER_MOTOR_REV = 1120;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = 1.5;    // This is > 1.0 if motors are geared up ____  Using OVerdrive gearing with Pico Uno boxes  40 gear to 35 gear over-drive
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

 @Override
 public void runOpMode() throws InterruptedException {

     robot2.init(hardwareMap);
     setupIMU();

     telemetry.addData("Mode ", "Waiting for start");
     telemetry.addData("imu calibrations status", imu.getCalibrationStatus().toString());
     telemetry.update();


     // Set the Initial; Blinkin Color Schema to Aqua (Represents CS9977 Team Colors
     robot2.blinkinLedDriver.setPattern(pattern);
     robot2.pattern = RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE;


     PIDDriveStrafeRight(1,0,48);    // Strafe Right at full power,  0 Deg angle, 48" Distance -- TEST ONLY not for Competition
     PIDDriveForward(1, 0, 48);      // Drive Forward at full power, 0 Deg angle, 40" Distance -- TEST ONLY not for Competition
     PIDDrivebackward(1,0,48);       // Drive Backward  at full power, 0 Deg angle, 48" Distance -- TEST ONLY not for Competition

     //  At this point of the testing, the robot should be back against the wall.   Any margin of error, means we need to tune the PID values


 } // Ends runOpMode

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
            robot2.DriveRightFront.setPower(speed);
            robot2.DriveLeftFront.setPower(speed);
            robot2.DriveRightRear.setPower(speed + correction);
            robot2.DriveLeftRear.setPower(speed);
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

}
