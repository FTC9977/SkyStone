package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="ConceptChangePID", group = "concept")

public class ConceptChangePID extends LinearOpMode {

    // Our DC Motors
    public DcMotorEx DriveLeftFront = null;
    public DcMotorEx DriveLeftRear = null;
    public DcMotorEx DriveRightFront = null;
    public DcMotorEx DriveRightRear = null;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    public void runOpMode() {
        // Get referece to DC Motors
        // since we are uisng the Expansion hub, cast this motor to DcMotorEx Object

        DriveLeftFront = (DcMotorEx)hardwareMap.get(DcMotor.class, "LF");
        DriveLeftRear = (DcMotorEx)hardwareMap.get(DcMotor.class, "LR");
        DriveRightFront = (DcMotorEx)hardwareMap.get(DcMotor.class, "RF");
        DriveRightRear = (DcMotorEx)hardwareMap.get(DcMotor.class, "RR");


        // Wait for Start
        waitForStart();

        // get the PID coefficients for the RUN_USING_ENCODER modes.

        PIDFCoefficients pidOrigLR = DriveLeftRear.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidOrigLF = DriveLeftFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidOrigRR = DriveRightRear.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidOrigRF = DriveRightFront.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //Change Coo

    }
}
