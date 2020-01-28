package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.HardwareMap.Robot;
import org.firstinspires.ftc.teamcode.HardwareMap.RobotWebcam;
import org.firstinspires.ftc.teamcode.HardwareMap.skyHardwareMap2;




@Autonomous(name="AutoVisionWeb", group="csweb")

public class VisionAutoWeb extends RobotWebcam {

    VuforiaStuff.skystonePos pos;
    int stoneDiff, stoneONE, stoneTWO;
    skyHardwareMap2 robot2 = new skyHardwareMap2();     //  Setup file for all DC Motors, IMU, etc...  ** Using this HwMap only for Test robot.  Competition robot will use skyHardwareMap();

    ElapsedTime runtime = new ElapsedTime();

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */



    // Created Rev Robotics BlinkIN instances

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;



    //Create IMU Instance
    PIDController pidDrive, pidRotate;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    //Define Drivetrain Variabeles

    static final double COUNTS_PER_MOTOR_REV = 537.6;                           // GoBilda 5202 YellowJacket 312RPM Motor
    static final double DRIVE_GEAR_REDUCTION = 1;                             // This is > 1.0 if motors are geared up ____  Using OVerdrive gearing with Pico Uno boxes  40 gear to 35 gear over-drive
    static final double WHEEL_DIAMETER_INCHES = 4.0;                            // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);



    @Override
    public void runOpMode() throws InterruptedException {
        robot2.init(hardwareMap);
        setupIMU();
        sleep(500);
        robotInit();         // Used to Setup Vuforia Parameters




        pos = vuforiaStuff.vuforiascan(true, false);
        switch (pos) {
            case LEFT:
                telemetry.addLine("Skystone position is LEFT");
                telemetry.update();
                //sleep(5000);
                stoneDiff = 0;
                stoneONE = 1;           // First Skystone is in Position 1
                stoneTWO = 4;           // Second Skystone is in Position 4 (2nd Left Position)

                // Blinked in:  Change color Flashing GREEN to indicate we found the stone
                break;
            case CENTER:
                telemetry.addLine("Skystone position is CENTER");
                telemetry.update();
                //sleep(5000);
                stoneDiff = 16;
                stoneONE = 2;           // First Skystone is in Position 2
                stoneTWO = 5;           // Second Skytone is in Position 5 (2nd Center Position)
                // Blinked in:  Change color Flashing GREEN to indicate we found the stone
                break;
            case RIGHT:
                telemetry.addLine("Skystone Position is RIGHT");
                telemetry.update();
                sleep(500);
                stoneDiff = 20;
                stoneONE = 3;           // First Skystone is in Position 3
                stoneTWO = 6;           // Second Skystone is in Postion 6 (2nd Right Positon)
                // Blinked in:  Change color Flashing GREEN to indicate we found the stone
                break;
        }





        /*if (pos == VuforiaStuff.skystonePos.LEFT) {
            telemetry.addLine("Were in the IF poss == VuforiaStuff.skystonePos.LEFT section");
            telemetry.update();
        } else {
            telemetry.addLine("Were in the 'else' portion of the IF poss == VuforiaStuff.skystonePos.LEFT section");
            telemetry.update();
        } */


        // SET INITIAL BlinkIn Color to Chasing Rainbow.


        // Setup Drive Commands to get to 1 out of 6 stone Positions


        if(stoneONE == 1) {
            telemetry.addLine("Driving to Skystone Position 1");
            DbgLog.msg("--------------");
            DbgLog.msg("   Driving to Skystone Position 1");
            DbgLog.msg("--------------");
            
            skystonePos1();             // Call Method to drive to position 1

        } else if (stoneONE == 2) {
            telemetry.addLine(" Driving to Skystone Position 2");
            DbgLog.msg("--------------");
            DbgLog.msg("   Driving to Skystone Position 2");
            DbgLog.msg("--------------");
            skystonePos2();             // Call Method to drive to position 1

        } else {
            telemetry.addLine("Driving to Skystone Position 3");
            DbgLog.msg("--------------");
            DbgLog.msg("   Driving to Skystone Position 3");
            DbgLog.msg("--------------");
            skystonePos3();             // Call Method to drive to position 3

         }


        //PIDDrivebackward(.80,90,27);


    }     //  END OF MAIN PROGRAM


// Main Autonomous Methods Section



 /*
 *  Skystone Position Definitions
 *
 *
 *																								      |
 *	   																							      |
 *       																							  |
 *      [ bridge ]    [Position1]  [Position2] [Position 3]  [Position 4]  [Position 5] [Position 6]  |
 *  																							 	  |   Perimeter Wall
 *  																							 	  |
 *  																							   /  |
 *  																					  	      /	  |
 *  																					         /	  |
 *  																					        /	  |
 *  			--------------------------------------------------------------------------------------|
 *
 *
 * Autonomous Sequence of Events:
 *
 *
             Autonomous Sequence:


             1. Raise Linear Lift (if needed) so web camera can function.  Make sure to do this only after waitForStart(), to avoid a penalty

                UNLESS....  We can position the webcamera to see the stones ahead of time (before match officially begins), and make the
                            determination which positions the stones are really in..  This would eliminate the need to raise the lift.


                 raiseLift();   // Call this method to slightly raise the lift



             2. Use webcamera to read Skystone Position and return a value of:  LEFT, CENTER, RIGHT


             3. Now, using a switch Statement, we will direct the AS program to perform several Functions:

                   - Also set two variables:  StoneONE and StoneTWO, to keep Track of where the second stone is located

                   SkyStone  		StoneONE 	StoneTWO
                   Position 		Position 	Position
                   --------------------------------------
                      LEFT				1			4
                      CENTER			2			5
                      RIGHT				3			6


                   - Print Telemetry to the Driver Station telling them which stone they are driving to
                   - Change the BlinkIn LED color to indicate to the driver the same information (Visual Feedback)
                   - Call one of 6 Drive Methods, which are pre-programmed paths to get to the SkyStone Positions (1-6)
                        - Within these drive methods, we will call other methods for:
                            - Driving to Foundation
                            - Placing stone
                            - Moving Foundation
                            - Driving to Bridge
                            - Parking


             4.	End of Delivering 1 or 2 stones we should be parked at the bridge.




             SkyStone Position Methods:

              - These methods will containt the PID drive parameters to move from the field wall (starting Position) to the skystone.
              - Robot will start in the Quarry side, and our alliance partner in the build zone
              - This is becuase we want the web camera to calculate the Skystone Position head on.

             skystonePos1()
             skystonePos2()
             skystonePos3()
             skystonePos4()
             skystonePos5()
             skystonePos6()




            Grabbing the stone Method:

                - This method will be used to grab the Skystone and raise the lift slightly off the ground for safe transport


            grabStone()



            Delivering Stone Methods:

                - These methods will move the robot from the Quarry side, under the bridge, and to the foundation side
                - There are six of these methods, since drive lengths from each of the SkyStone positions will be different, and we need to account for that.
                - We will next grab the foundation here as well. We will consider using a Limit Switch to detect the Foundation
                - Next, we will place the SkyStone on foundation (we dont care if it stacks or not, just places)
                - Lastly, we will release the foundation and drive to the bridge
                - End of AS mode

            deliverStone1()
            deliverStone2()
            deliverStone3()
            deliverStone4()
            deliverStone5()
            deliverStone6()
 *
 *
 */

    public void skystonePos1() {
        // This method will drive the robot to Skystone Position 1 (Closest to the bridge)
        // Blinked in:  Change color SOLID BLUE to indicate we successfully Drove to the stone
        PIDDrivebackward(.50,90,29);  // Gets Robot To Stone. -- WORKS
        grabStone();                                        // Grab Stone -- WORKS
        sleep(400);
        PIDDriveForward(.80 ,90, 8); // Drive Forward towards wall -- WORKS
       // sleep(500);
        RotateLeft(.8,20);
        resetAngle();
       // sleep(500);
        PIDDrivebackward(.5,90,70);
        RotateRight(.8,21);
        resetAngle();
        liftStone();
        PIDDrivebackward(.5,90,14);
        dropStone();
        PIDDriveForward(.5,90,14);
        lowerStone();
        RotateRight(.8,20);
        resetAngle();
        PIDDrivebackward(.8,90,35);



        /*  WORKAROUND....
        RotateRight(.80,21);

        PIDDriveForward(.80,90,40);
        PIDDrivebackward(1,90,15);

        */
    }



    public void skystonePos2() {
        // This method will drive the robot to Skyston Positon 2
        // Blinked in:  Change color SOLID BLUE to indicate we successfully Drove to the stone
        PIDDriveStrafeRight(.5,90,5);
        PIDDrivebackward(.50,90,29);  // Gets Robot To Stone. -- WORKS
        grabStone();                                        // Grab Stone -- WORKS
        sleep(400);
        PIDDriveForward(.80 ,90, 5); // Drive Forward towards wall -- WORKS

        // sleep(500);
        RotateLeft(.6,24);
        resetAngle();
        // sleep(500);
        PIDDrivebackward(.5,90,75);
        RotateRight(.8,21);
        resetAngle();
        liftStone();
        PIDDrivebackward(.5,90,12);
        dropStone();
        PIDDriveForward(.5,90,12);
        lowerStone();
        RotateRight(.8,20);
        resetAngle();
        PIDDrivebackward(.8,90,35);
        robot2.claw.setPosition(.9);

    }

    public void skystonePos3() {
        // This method will drive the robot to Skystone Position 3
        // Blinked in:  Change color SOLID BLUE to indicate we successfully Drove to the stone
        PIDDriveStrafeRight(.5,90,18);
        PIDDrivebackward(.5,90,29);
        grabStone();
        sleep(400);
        PIDDriveForward(.80,90,5);


        RotateLeft(.8,21);
        resetAngle();
        PIDDrivebackward(.5,90,85);
        RotateRight(.8,21);
        resetAngle();
        liftStone();
        PIDDrivebackward(.5,90,12);
        dropStone();
        PIDDriveForward(.5,90,12);
        lowerStone();
        RotateRight(.8,20);
        resetAngle();
        PIDDrivebackward(.8,90,40);
        robot2.claw.setPosition(.9);

    }

    public void skystonePos4() {
        // This method will drive the robot to Skystone Position 4
        // Blinked in:  Change color SOLID BLUE to indicate we successfully Drove to the stone
    }

    public void skystonePos5() {
        // This method will drive the robot to Skystone Position 5
        // Blinked in:  Change color SOLID BLUE to indicate we successfully Drove to the stone
    }

    public void skystonePos6() {
        // This method will drive the robot to Skystone Position 6
        // Blinked in:  Change color SOLID BLUE to indicate we successfully Drove to the stone
    }


    public void grabStone() {
        //  This method will be used to grab the stone  and prepare it for delivery
        robot2.claw.setPosition(.1);


    }

    public void liftStone(){
        robot2.LiftLeft.setPower(.75);
        robot2.LiftRight.setPower(.75);
        sleep(500);
        robot2.LiftRight.setPower(.1);
        robot2.LiftLeft.setPower(.1);
    }

    public void dropStone() {
        robot2.claw.setPosition(.9);
    }
    public void lowerStone() {
        robot2.LiftLeft.setPower(0);
        robot2.LiftRight.setPower(0);
    }

    public void foundationGrab(){
        // This method will be used to grab the Foundation an move it to the correct area
        // Change BlinkIN color to SOLID YELLOW to signal foundation move
        // First grab foundation
        // Rotate Foundation
        // Slide Foundation to Left

    }

    public void deliverStone1() {
        // This method will deliver the stone from Position 1
        // Rotation Robot to orient the stone facing the foundation
        // Then drive forward to deliver the stone
        // Grab Foundation and move                 call foundationGrab()
        // Place stone
        // Drive to Skybridge and park
    }

    public void deliverStone2() {
        // This method will deliver the stone from Position 2
        // Rotation Robot to orient the stone facing the foundation
        // Then drive forward to deliver the stone
        // Grab Foundation and move                 call foundationGrab()
        // Place stone
        // Drive to Skybridge and park
    }

    public void deliverStone3() {
        // This method will deliver the stone from Position 3
        // Rotation Robot to orient the stone facing the foundation
        // Then drive forward to deliver the stone
        // Grab Foundation and move                 call foundationGrab()
        // Place stone
        // Drive to Skybridge and park
    }

    public void deliverStone4() {
        // This method will deliver the stone from Position 4
        // Rotation Robot to orient the stone facing the foundation
        // Then drive forward to deliver the stone
        // Grab Foundation and move                 call foundationGrab()
        // Place stone
        // Drive to Skybridge and park
    }

    public void deliverStone5() {
        // This method will deliver the stone from Position 5
        // Rotation Robot to orient the stone facing the foundation
        // Then drive forward to deliver the stone
        // Grab Foundation and move
        // Place stone
        // Drive to Skybridge and park
    }

    public void deliverStone6() {
        // This method will deliver the stone from Position 6
        // Rotation Robot to orient the stone facing the foundation
        // Then drive forward to deliver the stone
        // Grab Foundation and move                 call foundationGrab()
        // Place stone
        // Drive to Skybridge and park
    }



    // These Methods below control all motor Movements via PID

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
            robot2.DriveLeftFront.setPower(speed + correction );
            robot2.DriveRightRear.setPower(speed + correction);
            robot2.DriveLeftRear.setPower(speed + correction );
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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //sleep(500);

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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        // Set RUN_TO_POSITION
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Stop Motors and set Motor Power to 0
        //PIDstopALL();
        robot2.DriveRightFront.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftRear.setPower(0);

    }  // END of PIDDriveStrafeLeft


    public void RotateLeft(double speed, int distance) {

        //Initialize Mecanum Wheel DC Motor Behavior
        //setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        // Reset Encoders
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);


        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        // Set RUN_TO_POSITION

        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(500);

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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot2.DriveRightFront.setPower(0);
        robot2.DriveRightRear.setPower(0);
        robot2.DriveLeftFront.setPower(0);
        robot2.DriveLeftRear.setPower(0);

        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        // Set RUN_TO_POSITION

        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(500);


    } // This brace closes out RotateLeft

    public void RotateRight(double speed, int distance) {

        //Initialize Mecanum Wheel DC Motor Behavior
        //setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        // Reset Encoders
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

        robot2.DriveRightFront.setTargetPosition(0);
        robot2.DriveRightRear.setTargetPosition(0);
        robot2.DriveLeftFront.setTargetPosition(0);
        robot2.DriveLeftRear.setTargetPosition(0);

        // Set RUN_TO_POSITION

        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(500);

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
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


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

    private void PIDstopALL () {
        // Reset Encoders
        robot2.DriveRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot2.DriveLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);

    }

}


