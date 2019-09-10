package org.firstinspires.ftc.teamcode;


import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutonomousData;
import org.firstinspires.ftc.teamcode.DriveTrain.PIDController;
import org.firstinspires.ftc.teamcode.HardwareMap.*;
import org.firstinspires.ftc.teamcode.Navigation.RevHubIMU;
import org.firstinspires.ftc.teamcode.DriveTrain.*;


@Autonomous(name="TestAS1", group = AutonomousData.OFFICIAL_GORUP)


public class TestAS extends LinearOpMode {

    // Declare Hardware
    csHardwareMap robot2 = new csHardwareMap();
    ElapsedTime runtime = new ElapsedTime();

    //Create IMU Instance
    PIDController pidDrive, pidRotate;
    MecanumDrive mecanum;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    RevHubIMU setupIMU;

    // X-position pixel value for center of robot
    public final static int ROBOT_CENTER_X = 374;     // Was 285


    static final double COUNTS_PER_MOTOR_REV = 1120;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = 1.0;    // This is > 1.0 if motors are geared up
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() throws InterruptedException {

        robot2.init(hardwareMap);
        setupIMU.calibrate();  // Setup, Initialize, and Calibrate IMU
        telemetry.addData("IMU Calib stuats = ", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Wait for Start

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status :", "Waiting for start command...");
            telemetry.update();
        }

        mecanum.PIDDriveForward(.4, 90, 4);
        mecanum.PIDDriveStrafeLeft(.4, 90, 4);
        mecanum.PIDDriveBackwards(.4, 90, 4);
        mecanum.PIDDriveStrafeRight(.4, 90, 4);


    }

}




