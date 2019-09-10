package org.firstinspires.ftc.teamcode.DriveTrain;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.Navigation.RevHubIMU;
import org.firstinspires.ftc.teamcode.HardwareMap.MRGyro;
import com.qualcomm.hardware.modernrobotics.*;

import org.firstinspires.ftc.teamcode.HardwareMap.csHardwareMap;
// import org.firsnspires.ftc.teamcode.FieldMapping.FieldElement;
// import org.firstinspires.ftc.teamcode.FieldMapping.Vector;


/* MecanumDrive.java

Purpose:

This class file contains the individual Mecanum Wheel motions use during Autonomous Mode.  This includes PID Methods



Revision History:

7/28/19     Eric        Initial File Creation.
 */


public class MecanumDrive {

    //Declare Hardware
    //ssHardwareMap robot2 = new ssHardwareMap();


    // Motor Definitions
    public DcMotor DriveLeftFront;
    public DcMotor DriveLeftRear;
    public DcMotor DriveRightFront;
    public DcMotor DriveRightRear;

    // Motor Encoder Definitions
    public Encoder DriveLeftFrontEncoder;
    public Encoder DriveLeftRearEncoder;
    public Encoder DriveRightFrontEncoder;
    public Encoder DriveRightRearEncoder;

    // Navigation and Positional Configuration Components
    //public Vector robotPos;                 // Robots Position on the playing field
    public double robotAngle;               // Angle relative to (0,0) on the field
    public RevHubIMU imu;                   // Added: 1/18/19 -- RevIMU Definition
    private double initialIMUHeading;       // Added: 1/18/19 -- IMU When the roboti first hits the floor
    private double initialRobotAngle;       // Added: 1/18/19 -- Manually set robot angle when the robot first hits the floor
    PIDController pidDrive, pidRotate;



    // Define Gamepads

    private Gamepad gamepad1;



    // Define Modern Robotics External GYRO
    public MRGyro gyroSensor;

    // Define Autonomous Based VARIABLES
    private LinearOpMode autonomous = null;     // stays nul unless it is used for Autonomous Mode
    private long startTime;

    // Variables to regulate the Maximum Motor Power
    private final double MAX_DRIVE_POWER = 1;
    private double boost;

public MecanumDrive(DcMotor LF, DcMotor LR, DcMotor RF, DcMotor RR, RevHubIMU hubIMU, Gamepad gamepad, double wheelDiam) {

    //robotPos = new Vector(0,0);     // Set the default position of the robot.  Used later in AS Mode
    robotAngle = 0;                 // Set the default angle of the robot. Used Later in AS Mode

    DriveLeftFront = LF;
    DriveLeftRear = LR;
    DriveRightFront = RF;
    DriveRightRear = RR;

    //gyroSensor = gyro;
    imu = hubIMU;
    this.gamepad1 = gamepad;

    DriveLeftFrontEncoder = new Encoder(DriveLeftFront, AutonomousData.NEVEREST_ENCODER40, wheelDiam);
    DriveLeftRearEncoder = new Encoder(DriveLeftRear, AutonomousData.NEVEREST_ENCODER40, wheelDiam);
    DriveRightFrontEncoder = new Encoder(DriveRightFront, AutonomousData.NEVEREST_ENCODER40, wheelDiam);
    DriveRightRearEncoder = new Encoder(DriveRightRear, AutonomousData.NEVEREST_ENCODER40, wheelDiam);

    boost = 1;
}
   // public void setRobotPos(Vector pos) {
     //   robotPos = pos;
   // }

    public void setRobotAngle(double angle){
        robotAngle = angle;
    }

    public void setInitialRobotAngle(double angle) {
        initialRobotAngle = angle;
    }

    public void setInitialIMUHeading() {
        initialIMUHeading = imu.getHeading();
    }

    public void initHardware() {
        DriveLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveLeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        DriveRightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        DriveRightFront.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set DC Motor ZeroPowerBehavior to BRAKE

        DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderSetup();
    }

    public void encoderSetup() {
        DriveLeftFrontEncoder.setup();
        DriveLeftRearEncoder.setup();
        DriveRightFrontEncoder.setup();
        DriveRightRearEncoder.setup();
    }

    public void setPowers(double lp, double rp) {
        DriveLeftFront.setPower(lp);
        DriveLeftRear.setPower(lp);
        DriveRightFront.setPower(rp);
        DriveRightRear.setPower(rp);
    }


    public void setStartTime(long time) {
        startTime = time;
    }

    public void setAuto(LinearOpMode auto) {
        autonomous = auto;
    }

    // This is used to break all "while" loops, when an OpMode Stops
    private boolean autoRunning() {
        return System.currentTimeMillis() - startTime <= AutonomousData.TIME_LIMIT && !autonomous.isStopRequested();
    }


    /*
     *  The following controls the robots ability to drive Forwards or Backwards (x) ammount
     *  of distance using the Motor Encoders
     *
     *  Parameter Definitions:
     *   direction :   1 = Forward Direction,    -1 = Backwards
     *   distance  : The distance, in inches, for robot to travel over
     *   power     :  Power to apply to all 4 mecanum wheel motors
     */



    public void driveDistance(int direction, double distance, double pow) {

        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();

        DriveLeftFrontEncoder.runToPosition();
        DriveLeftRearEncoder.runToPosition();
        DriveRightFrontEncoder.runToPosition();
        DriveRightRearEncoder.runToPosition();

        DriveLeftFrontEncoder.setTarget(direction * distance);
        DriveLeftRearEncoder.setTarget(direction * distance);
        DriveRightFrontEncoder.setTarget(direction * distance);
        DriveRightRearEncoder.setTarget(direction * distance);

        setPowers(direction * pow, direction * pow);

        while (DriveLeftRear.isBusy() && DriveLeftFront.isBusy() && DriveRightFront.isBusy() && DriveRightRear.isBusy() && autoRunning()) {
            // Wait, as all 4 motors are currently busy
        }

        setPowers(0, 0);

        DriveLeftFrontEncoder.runWithout();
        DriveLeftRearEncoder.runWithout();
        DriveRightFrontEncoder.runWithout();
        DriveRightRearEncoder.runWithout();

    }

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
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior

        setZeroPowerBrakes();   // Set DC Motor Brake Behavior
        //DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();
        //DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
        //DriveRightFront.setPower(0);
        //DriveLeftFront.setPower(0);
        //DriveRightRear.setPower(0);
        //DriveLeftRear.setPower(0);

        double InchesMoving = (distance * imu.CPI());


        // Set Target
        DriveRightFront.setTargetPosition((int) InchesMoving);
        DriveLeftFront.setTargetPosition((int) InchesMoving);
        DriveRightRear.setTargetPosition((int) InchesMoving);
        DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(speed + correction);
            DriveLeftFront.setPower(speed + correction);
            DriveRightRear.setPower(speed + correction);
            DriveLeftRear.setPower(speed + correction);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        //DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDstopALL();

    }   // END OF PIDDriveForward


    public void PIDDriveBackwards (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving
        imu.resetAngle();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());

        // Set Target
        DriveRightFront.setTargetPosition((int) -InchesMoving);
        DriveLeftFront.setTargetPosition((int) -InchesMoving);
        DriveRightRear.setTargetPosition((int) -InchesMoving);
        DriveLeftRear.setTargetPosition((int) -InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(-speed + correction);
            DriveLeftFront.setPower(-speed + correction);
            DriveRightRear.setPower(-speed);
            DriveLeftRear.setPower(-speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveBackwards

    public void PIDDriveStrafeRight (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving
        imu.resetAngle();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());


        // Set Target
        DriveRightFront.setTargetPosition((int) -InchesMoving);
        DriveLeftFront.setTargetPosition((int) InchesMoving);
        DriveRightRear.setTargetPosition((int) InchesMoving);
        DriveLeftRear.setTargetPosition((int) -InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(-speed);
            DriveLeftFront.setPower(-speed);
            DriveRightRear.setPower(-speed + correction);
            DriveLeftRear.setPower(-speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveStrafeRight

    public void PIDDriveStrafeLeft (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving
        imu.resetAngle();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());

        // Set Target
        DriveRightFront.setTargetPosition((int) InchesMoving);
        DriveLeftFront.setTargetPosition((int) -InchesMoving);
        DriveRightRear.setTargetPosition((int) -InchesMoving);
        DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(speed + correction);
            DriveLeftFront.setPower(speed);
            DriveRightRear.setPower(speed);
            DriveLeftRear.setPower(speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveStrafeLeft

    public void PIDDriveDiagnolLeftForward (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());

        // Set Target
        DriveRightFront.setTargetPosition((int) InchesMoving);
        //DriveLeftFront.setTargetPosition((int) -InchesMoving);
        //DriveRightRear.setTargetPosition((int) -InchesMoving);
        DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(-speed + correction);
            DriveLeftFront.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftRear.setPower(-speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveDiagnolLeftForward

    public void PIDDriveDiagnolRightForward (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());

        // Set Target
        //DriveRightFront.setTargetPosition((int) InchesMoving);
        DriveLeftFront.setTargetPosition((int) InchesMoving);
        DriveRightRear.setTargetPosition((int) InchesMoving);
        //DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(0);
            DriveLeftFront.setPower(-speed + correction);
            DriveRightRear.setPower(-speed);
            DriveLeftRear.setPower(0);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveDiagnolRightForward


    public void PIDDriveLeftDiagnolBack (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());

        // Set Target
        DriveRightFront.setTargetPosition((int) InchesMoving);
        DriveLeftFront.setTargetPosition((int) -InchesMoving);
        DriveRightRear.setTargetPosition((int) -InchesMoving);
        DriveLeftRear.setTargetPosition((int) InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(0);
            DriveLeftFront.setPower(-speed);
            DriveRightRear.setPower(-speed + correction);
            DriveLeftRear.setPower(0);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveLeftDiagnolBack



    public void PIDDriveRightDiagnolBack (double speed, double angle, int distance) {

        // Setup straight line
        // Setup Straight Line Driving

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, speed);
        pidDrive.setInputRange(-angle, angle);
        pidDrive.enable();


        // Use PID with imu input to drive in a straight line.
        double correction = pidDrive.performPID(imu.getAngle2());



        //Initialize Mecanum Wheel DC Motor Behavior
        setZeroPowerBrakes();   // Set DC Motor Brake Behavior

        //  Reset Encoders:    Alternate way:  DriveRightFrontEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Set RUN_TO_POSITION
        DriveRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveLeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        autonomous.sleep(500);

        // Stop Motors and set Motor Power to 0
        PIDstopALL();


        double InchesMoving = (distance * imu.CPI());

        // Set Target
        DriveRightFront.setTargetPosition((int) -InchesMoving);
        DriveLeftFront.setTargetPosition((int) InchesMoving);
        DriveRightRear.setTargetPosition((int) InchesMoving);
        DriveLeftRear.setTargetPosition((int) -InchesMoving);


        while (DriveRightFront.isBusy() && DriveRightRear.isBusy()
                && DriveLeftFront.isBusy() && DriveLeftRear.isBusy()) {


            //Set Motor Power  - This engages the Motors and starts the robot movements
            DriveRightFront.setPower(-speed + correction);
            DriveLeftFront.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftRear.setPower(-speed);
        }    // This brace closes out the while loop

        //Reset Encoders
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
        DriveLeftFrontEncoder.reset();
        DriveLeftRearEncoder.reset();

        // Stop Motors and set Motor Power to 0
        PIDstopALL();
    }  // End of PIDDriveRightDiagnolBack

    public void PIDRotate(int degrees, double power) {

        imu.resetAngle();

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0,90);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();


        // getAngle() returns "+" when rotating counter clockwise (left) and "-" when rotating clockwise (right)

        if(degrees < 0) {
            // On right turn we have to get off zero first
            while(imu.getAngle2() == 0) {
                DriveLeftFront.setPower(-power);
                DriveLeftRear.setPower(-power);
                DriveRightFront.setPower(power);
                DriveRightRear.setPower(power);
                autonomous.sleep(100);
            }
            do
            {
                power = (pidRotate.performPID(imu.getAngle2() + .50)); // power will be "-" on right turn
                DriveLeftFront.setPower(power);
                DriveLeftRear.setPower(power);
                DriveRightFront.setPower(-power);
                DriveRightRear.setPower(-power);
            } while ( !pidRotate.onTarget());
        }
        else  // Turn Left
            do
            {
                power = (pidRotate.performPID(imu.getAngle2() + .50));  // Power will be "+" on left turn
                DriveLeftFront.setPower(power);
                DriveLeftRear.setPower(power);
                DriveRightFront.setPower(-power);
                DriveRightRear.setPower(-power);
            } while(!pidRotate.onTarget());


        // Turn Motors Off
        PIDstopALL();

        // Wait for Rotation to Stop
        autonomous.sleep(500);

        // Reset Angle tracking on new Heading
        imu.resetAngle();

    }

    /*
     * This method will cause the Robot to Turn (x) ammount of degrees using the MR Gyro
     *
     * Parameters:
     *   Degrees  :  How many degrees we want the robot to turn
     *   Right    :   If set to "True", then turn to the right
     *                If set to "False", then turn to the left
     */

    public void turn(int degrees, boolean right) throws  InterruptedException {    // 1/18/19 - Added throws InterruptedException
        gyroSensor.zero();
        encoderSetup();
        int currAngle = Math.abs(gyroSensor.getAngle());  // Use getAngle() because it returns angle robot has turned from origin
        double startPow = 0.3;  // Starting Power                   Added: 1/18/19
        double pow;             // power applied to motors          Added: 1/18/19
        double prop;            // proportion of angle completed    Added: 1/18/19

        // DEBUG Telemetry for:   currAngle
        //autonomous.telemetry.addData("turn_CurrAngle is ", currAngle);
        //autonomous.telemetry.addData("turn_Degrees Passed = ", degrees);


        while (currAngle < degrees && autoRunning()) {
            prop = (double) currAngle / degrees;
            pow = startPow * Math.pow((prop -1), 2) ;  //  orginaly 0.6 at qualifier


            // Apply power to motors and update currAngle
            if (right)
                setPowers(pow, -pow);
            else
                setPowers(-pow, pow);
            currAngle = Math.abs(gyroSensor.getAngle());

            // DEBUG Telemetry
            //autonomous.telemetry.addData("turn_Updated current Angle is: ", currAngle);
            //autonomous.telemetry.update();
        }
        setPowers(0, 0);

        // 1/18/19 Added Telemetry Data Output tp Driver Station Phones
        autonomous.telemetry.addData("Gyro Sensor Reading", gyroSensor.getAngle());
        autonomous.telemetry.update();


        // Updates the robot angle based on turn
        Thread.sleep(200);
        updateAngleFromIMU();                       // 1/18/19 Added;  Removed updateAnleFromGyro();

        // DEBUG Telemetry
        //autonomous.telemetry.addData("turn_Robot Angle", robotAngle);

    }


    // Positional Updating Methods
    public void updatePosFromEncoders () {

        /*  Changed temRobotAngle variable: 1/18/19
         *
         *   OLD = int tempRobotAngle = robotAngle > 180 ? -(360 - robotAngle): robotAngle;
         */
        int tempRobotAngle = robotAngle > 180 ? -(360 - (int) Math.round(robotAngle)) : (int) Math.round(robotAngle);
        double theata = MRGyro.convertToRadians(tempRobotAngle);
        double dist = ((DriveLeftFrontEncoder.linDistance() + DriveLeftRearEncoder.linDistance() + DriveRightFrontEncoder.linDistance() + DriveRightRearEncoder.linDistance()) / 2);  // Distance travelledd according to the Motor Encoders
        //setRobotPos(robotPos.sum(new Vector(dist * Math.cos(theata), dist * Math.sin(theata))));  // Added 1/3/19
        DriveLeftFrontEncoder.reset();;
        DriveLeftRearEncoder.reset();
        DriveRightFrontEncoder.reset();
        DriveRightRearEncoder.reset();
    }


    public void updateAngleFromGyro() {
        setRobotAngle((360 + robotAngle - gyroSensor.getAngle()) % 360);
        gyroSensor.zero();                  //  added 1/3/9

        // DEBUG Telemetry
        autonomous.telemetry.addData("Robot Angle from Gyro", robotAngle);   // Updated 1/4/19
        autonomous.telemetry.update();

    }
    public void updateAngleFromIMU() {
        setRobotAngle((360 + initialRobotAngle - (imu.getHeading() - initialIMUHeading)) % 360);
    }

    // Accessor methods
    public double getLeftFrontPow() {
        return DriveLeftFront.getPower();
    }

    public double getLeftRearPow() {
        return DriveLeftRear.getPower();
    }

    public double getRightFrontPow() {
        return DriveRightFront.getPower();
    }

    public double getRightRearPow() {
        return DriveRightRear.getPower();
    }

    public Encoder getLeftFrontEncoder() {
        return DriveLeftFrontEncoder;
    }

    public Encoder getDriveLeftRearEncoder() {
        return DriveLeftRearEncoder;
    }

    public Encoder getDriveRightFrontEncoder() {
        return DriveRightFrontEncoder;
    }

    public Encoder getDriveRightRearEncoder() {
        return DriveRightRearEncoder;
    }

    public MRGyro getGyro() {
        return gyroSensor;
    }

    public double getBoost() {
        return boost;
    }


    private void setZeroPowerBrakes() {
        //Initialize Mecanum Wheel DC Motor Behavior
        DriveRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void PIDstopALL() {
        DriveRightFront.setPower(0);
        DriveLeftFront.setPower(0);
        DriveRightRear.setPower(0);
        DriveLeftRear.setPower(0);
    }


}
