package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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


@Autonomous(name="Capstone_TEST", group = "captest")

public class Capstone_TEST extends LinearOpMode {


    // Decalre hardware

    //skyHardwareMap robot2 = new skyHardwareMap();
    ElapsedTime runtime = new ElapsedTime();


    private static DcMotor motor1;


    // REV Robotics Color Sensor + Distance Sensor Rev v3 Definitions
    ColorSensor sensorColor = null;
    DistanceSensor sensorDistance = null;
    boolean capstoneDetected = false;

    static final double COUNTS_PER_MOTOR_REV = 753.2;   // Andymark 40 Motor Tick Count
    static final double DRIVE_GEAR_REDUCTION = .69;    // This is > 1.0 if motors are geared up ____  Using OVerdrive gearing with Pico Uno boxes  40 gear to 35 gear over-drive
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // For figuring out circumfrance
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        //robot2.init(hardwareMap);

        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setPower(0);


        // Configuration setup for Capstone Detection using REV Color/Distance Sensor v3
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


        // wait for the start button to be pressed.


        // This chunk of code gets around the Motorola E4 Disconnect bug.  Should be fixed in SDK 5.3, but adding it as a "backup - JUST IN CASE!!!"
        //
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        waitForStart();


        // setup PID Drive Commands to get to first Capstone closest to bridge and align robot with color sensor on first Capstone

        // Robot should now be infront of First capstone.  Invoke the Detection Method


        while (opModeIsActive()) {


            if (capDetect() == true && sensorDistance.getDistance(DistanceUnit.INCH) <= 2.0) {
                telemetry.addLine("Capstone is found");
                telemetry.addData("Distance(inches)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.INCH)));
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.update();
                //sleep(500);
                //motor1.setPower(.85);
                //sleep(500);
                break;

            } else
                telemetry.addLine("Capstone was not found,.....");
            telemetry.addData("Distance(inches)", String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.INCH)));
            telemetry.update();
            motor1.setPower(.25);
            sleep(500);
            motor1.setPower(0);
        }

        telemetry.addLine("We are at the end of the If..Then... Else");



    }

       /*
        int capCount = 0;
        while (opModeIsActive() && capCount != 6) {
            // Capstone detection attempt #1
            capDetect(false);    // Call Capstone Detection Method
            if (capDetect(capstoneDetected = true)) {
                // Call Ingest method
                // Drive to foundtation
                // Place Block
                // Move Foundation
                // Drive to centerline and stop
                telemetry.addLine("Capstone Found");
                telemetry.update();
                capCount = 6;
            } else
                capCount = capCount + 1;  // Increase Capcount by 1
                 motor1.setPower(.50);    // straff right and try againp0/;
                 sleep(1000);
                 motor1.setPower(0);
            // Capstone dection attempt #2
            capDetect(false); //Call Capstone Detection Method
            if (capDetect(capstoneDetected = true)) {
                // Call Ingest method
                // Drive to foundtation
                // Place Block
                // Move Foundation
                // Drive to centerline and stop
                telemetry.addLine("Capstone Found");
                telemetry.update();
            } else
                capCount = capCount + 1;  // Increase Capcount by 1
            //PIDDriveStrafeRight(.50, 90, 12);   // straff right and try again
            // Capstone dection attempt #3
            capDetect(false); //Call Capstone Detection Method
            if (capDetect(capstoneDetected = true)) {
                // Call Ingest method
                // Drive to foundtation
                // Place Block
                // Move Foundation
                // Drive to centerline and stop
                telemetry.addLine("Capstone Found");
                telemetry.update();
            } else
                capCount = capCount + 1;  // Increase Capcount by 1
            //PIDDriveStrafeRight(.50, 90, 12);   // straff right and try again
            // Capstone dection attempt #4
            capDetect(false); //Call Capstone Detection Method
            if (capDetect(capstoneDetected = true)) {
                // Call Ingest method
                // Drive to foundtation
                // Place Block
                // Move Foundation
                // Drive to centerline and stop
                telemetry.addLine("Capstone Found");
                telemetry.update();
            } else
                capCount = capCount + 1;  // Increase Capcount by 1
            //PIDDriveStrafeRight(.50, 90, 12);   // straff right and try again
            // Capstone dection attempt #5
            capDetect(false); //Call Capstone Detection Method
            if (capDetect(capstoneDetected = true)) {
                // Call Ingest method
                // Drive to foundtation
                // Place Block
                // Move Foundation
                // Drive to centerline and stop
                telemetry.addLine("Capstone Found");
                telemetry.update();
            } else
                capCount = capCount + 1;  // Increase Capcount by 1
            //PIDDriveStrafeRight(.50, 90, 12);   // straff right and try again
            // Capstone dection attempt #6
            capDetect(false); //Call Capstone Detection Method
            if (capDetect(capstoneDetected = true)) {
                // Call Ingest method
                // Drive to foundtation
                // Place Block
                // Move Foundation
                // Drive to centerline and stop
                telemetry.addLine("Capstone Found");
                telemetry.update();
            } else
                capCount = capCount + 1;  // Increase Capcount by 1
            //PIDDriveStrafeLeft(1, 90, 96);  // Could not detect Capston.  Something went wrong.  Gracefully move to the center line and park
        }
        */


    // }


    // This section begins the programs METHOD(s) classes
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
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);


        //  This is the algorithm to Normailze the Blue channel.
    /*
      Concept:
         Normailize the Blue channel.    Multiple the Red * Gree deviced by Blue^2
         If the Color Condition <= RevCondition # (3); then we are seeing the capstone (black)
          otherwise we are seeing soemthing other than black, and not seeing the capston
          Courtesy of FTC 5898 Youtube Explaination.  (https://www.youtube.com/watch?v=i0AskHFkZ94)
     */
        ColorCondition = (sensorColor.red() / sensorColor.blue() * (sensorColor.green() / sensorColor.blue()));

        if(ColorCondition <= RevCondition) {
            //telemetry.addLine("capstone detected");
            telemetry.addData("Color Condition equals ", ColorCondition);
            //telemetry.addData("Red  ", sensorColor.red());
            //telemetry.addData("Green", sensorColor.green());
            //telemetry.addData("Blue ", sensorColor.blue());
            return true;

        } else
            //telemetry.addLine("Capstone Not Detected");
            return false;
    }
}