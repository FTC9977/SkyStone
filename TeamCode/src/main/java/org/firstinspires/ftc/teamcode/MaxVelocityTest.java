package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/*
  *  This program will assist the user in Basic PIDF tuning.
  *
  * Concept:
  *    You can establish basic PIDF values for your drivetrain by plugging a simple measurement into a formula.
  *    This will be suffucuent for most use cases.  The measurement you need is the "MAXIMIM MOTOR VELOCITY" of your robot.
  *
  *    To get this,  run a maximum velocity measurement OpMode with a full battery
  *
  *  IMPORTANT:
  *    In this OpMode, it is important that you set your motors mode to "RUN_WITHOUT_ENCODER", and the power set to 1.
 */


@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    public DcMotorEx DriveLeftFront = null;
    public DcMotorEx DriveLeftRear = null;
    public DcMotorEx DriveRightFront = null;
    public DcMotorEx DriveRightRear = null;

    double currentVelocityLF, currentVelocityRF, currentVelocityLR, currentVelocityRR;
    double maxVelocity = 0.0;


    @Override
    public void runOpMode() {



        DriveLeftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, "LF");
        DriveLeftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, "LR");
        DriveRightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, "RF");
        DriveRightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, "RR");

        DriveLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        DriveLeftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        DriveRightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        DriveRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();

        while (opModeIsActive()) {

            DriveRightFront.setPower(1);
            DriveRightRear.setPower(1);
            DriveLeftFront.setPower(1);
            DriveLeftRear.setPower(1);

            currentVelocityLF = DriveLeftFront.getVelocity();
            currentVelocityRF = DriveRightFront.getVelocity();
            currentVelocityLR = DriveLeftRear.getVelocity();
            currentVelocityRR = DriveRightRear.getVelocity();

            if ((((currentVelocityLF > maxVelocity) && (currentVelocityLR > maxVelocity) && (currentVelocityRF > maxVelocity) && (currentVelocityRR > maxVelocity)))) {
                maxVelocity = currentVelocityRF;
            }

            telemetry.addData("current DriveLeftFront Velocity: ", currentVelocityLF);
            telemetry.addData("current DriveLeftRear Velocity: ", currentVelocityLR);
            telemetry.addData("current DriveRightFront Velocity: ", currentVelocityRF);
            telemetry.addData("curret DriveRightRear Velocity: ", currentVelocityRR);
            telemetry.addData("Maxmimum Velocity: ", maxVelocity);      // This is the value we need for performing our calculations
            telemetry.update();

            sleep(10000);  // Run for 10 seconds

            // Turn off motors
            DriveRightFront.setPower(0);
            DriveRightRear.setPower(0);
            DriveLeftFront.setPower(0);
            DriveLeftRear.setPower(0);

            DriveLeftRear.setVelocityPIDFCoefficients(1.26,.126,0,12.6);
            DriveLeftRear.setPositionPIDFCoefficients(5.0);

        }
    }
}
