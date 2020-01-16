package org.firstinspires.ftc.teamcode.Autonomous;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.HardwareMap.CSimuHardwareMap;

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
 */


@Autonomous(name="ENCODER_TESTING", group = "pid-test")

public class ENCODER_TESTING extends LinearOpMode {


    // Decalre hardware


    CSimuHardwareMap robot2 = new CSimuHardwareMap();
    ElapsedTime runtime = new ElapsedTime();






    //Create IMU Instance
    PIDController pidDrive, pidRotate;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    //Define Drivetrain Variabeles

    static final double COUNTS_PER_MOTOR_REV = 753.2;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = .5;    // This is > 1.0 if motors are geared up ____  Using OVerdrive gearing with Pico Uno boxes  40 gear to 35 gear over-drive
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

 @Override
 public void runOpMode() throws InterruptedException {

     robot2.init(hardwareMap);
     setupIMU();



     // This chunk of code gets around the Motorola E4 Disconnect bug.  Should be fixed in SDK 5.3, but adding it as a "backup - JUST IN CASE!!!"
     //
     while (!opModeIsActive() && !isStopRequested()) {
         telemetry.addData("status", "waiting for start command...");
         telemetry.update();
     }

     waitForStart();

     // Using Advacned PID Tuning via REV COntrol Hub
     //   Reference URL: https://github.com/FIRST-Tech-Challenge/SkyStone/wiki/Changing-PID-Coefficients
     //
     // Example Command:
     // robot2.DriveLeftFront.setVelocityPIDFCoefficients(1.26, .126, 0, 12.6);

     // 90 Degrees = Straight line



  /*  GoBilda Motor Encoder Mis-Wiring Testing Statement
   *
   *   There is an issue with GoBilda Manufacturing proccess where the Motor Encoder wires were
   *   incorrectly mis-wired.
   *
   *   Symptoms include:
   *    - motor Encoder tick counts decreasing vs increasing
   *    - irregular driving patterns during Autonomous Code Execution
   *    - PID CONTROL Algorithms not functioning properly
   *
   *   FTC Forum Post identifying this issue:
   *    https://ftcforum.firstinspires.org/forum/ftc-technology/75167-warning-about-gobilda-motor-directions
   *
   *  To Test:
   *    1. Set robot up on blocks or Floor
   *    2. Run the Motors at slow speed (25% motor power)
   *    3. Instruct robot to run a LONG distance (were using 200" inches)
   *    4. Telemetry will be displayed on the Driver Station phone
   *        - Verify all 4 motors are incrementing in counts
   *        - Any Motors that are "decreasing" in values would indicate a motor that is mis-wired
   *
   *    5. Repair or Replace motor as needed
   *
   */

    // Command below assumes robot is on blocks, NOT ON THE FIELD!!!!!!!!!!!!!!


      PIDDriveForward(.25,90,200); // Drive Forward @ 25% power, 90 degrees, 200"



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

        // Note:  If your huv is mounted vertically, remap the IMU axes so that the z-axis points
        //        upward (normal to the floo) using a command like:
        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

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
            robot2.DriveLeftFront.setPower(speed + correction );
            robot2.DriveRightRear.setPower(speed + correction);
            robot2.DriveLeftRear.setPower(speed + correction );

            // This telemetry is being used to test the GOBILD Motor Encorder Mis-Wiring issues
            //
            //  All Motors  Encoder Ticks should be increasing, nNOT decreasing
            //    Any motors that are found to be decreasing has a mis-wired Motor Encoder
            telemetry.addData("DriveRightFront CPI: ", robot2.DriveRightFront.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("DriveRightRear CPI: ", robot2.DriveRightRear.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addLine(" ---------------------");
            telemetry.addData("DriveLeftFront CPI: ", robot2.DriveLeftFront.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("DriveLeftRear CPI: ", robot2.DriveLeftRear.getCurrentPosition()/COUNTS_PER_INCH);
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
        robot2.DriveRightFront.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) -InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) -InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (robot2.DriveRightFront.isBusy() && robot2.DriveRightRear.isBusy()
                && robot2.DriveLeftFront.isBusy() && robot2.DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            robot2.DriveRightFront.setPower(speed);
            robot2.DriveLeftFront.setPower(speed + correction);
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
        robot2.DriveRightFront.setTargetPosition((int)- InchesMoving);
        robot2.DriveLeftFront.setTargetPosition((int) InchesMoving);
        robot2.DriveRightRear.setTargetPosition((int) InchesMoving);
        robot2.DriveLeftRear.setTargetPosition((int) -InchesMoving);


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

}
