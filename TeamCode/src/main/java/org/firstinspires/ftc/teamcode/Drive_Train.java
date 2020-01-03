package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;


@TeleOp(name="DriveTrain", group="calebs_robot")
//@Disabled
public class Drive_Train extends LinearOpMode {
    /*
    *  Thing to fix
    *
    *
    *       1. fix the strafing on the back two wheels
    *       2. add commands to use the arm
    *       3. chang the shafting on the arm
    *       4. lower the sen of the controller
    *
    *
    * */

    //skyHardwareMap robot2 = new skyHardwareMap();

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double ACCEPTINPUTTHRESHOLD = .15;
    private static final double SCALEDPOWER = 1; // The emphasis is in the current controller reading (vs. current motor power) on the drive train

    private static DcMotor leftFrontWheel;
    private static DcMotor leftBackWheel;
    private static DcMotor rightFrontWheel;
    private static DcMotor rightBackWheel;
    private static DcMotor LiftLeft;
    private static DcMotor LiftRight;
    private static DcMotor InRight;
    private static DcMotor InLeft;
    private static Servo   ArmServo;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT1NAME); // "LF";   // Left Front, LF l1     port 2
        leftBackWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT2NAME); // "LR";   // Left Rear, LR, l2      port 3
        rightFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHT1NAME); // "RF";  // Right Front, RF, r1  port 1
        rightBackWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHTNAME2); // "RR";  // Right Rear, RR, r2    port 0

        LiftLeft = hardwareMap.dcMotor.get(UniversalConstants.LiftLeft);
        LiftRight = hardwareMap.dcMotor.get(UniversalConstants.LiftRight);

        InLeft = hardwareMap.dcMotor.get(UniversalConstants.InLeft);
        InRight = hardwareMap.dcMotor.get(UniversalConstants.InRight);

        ArmServo = hardwareMap.servo.get(UniversalConstants.ArmServo);



        LiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);


       rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        InLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {


            while (gamepad1.a) {
                 LiftLeft.setPower(.75);
                 LiftRight.setPower(.75);

                 if(!gamepad1.a) {
                     LiftLeft.setPower(.02);
                     LiftRight.setPower(.02);
                     sleep(3000);
                     LiftRight.setPower(0);
                     LiftLeft.setPower(0);
                 }
            }


            if (gamepad1.dpad_down){
                InLeft.setPower(.8);
                InRight.setPower(.75);
            }else if (gamepad1.dpad_up){
                InLeft.setPower(-.8);
                InRight.setPower(-.75);
            }else if (gamepad1.dpad_left){
                InLeft.setPower(0);
                InRight.setPower(0);
            }


            if (gamepad1.right_bumper == true){
                ArmServo.setPosition(.1);
            }else if (gamepad1.right_bumper == false){
                ArmServo.setPosition(.7);
            }


           /*
            if(gamepad1.a == true) {

                LiftLeft.setPower(.50);
                LiftRight.setPower(.50);


            } else if (gamepad1.b == true) {
                LiftLeft.setPower(-.10);
                LiftRight.setPower(-.10);
            } else {
                LiftLeft.setPower(0);
                LiftRight.setPower(0);
            }
            */

            double inputY = Math.abs(-gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_y : 0;
            double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_x : 0;
            double inputC = Math.abs(-gamepad1.right_stick_x) > ACCEPTINPUTTHRESHOLD ? gamepad1.right_stick_x : 0;

            arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);




        /* This is old RR code, here for placement only...  Use this section to define
           actions during TelOp Play..

           For Example, using a sweeper arm to grab/pickup blocks/balls

           REPLACE THIS CODE BLOCK with SKYSTONE Requirements...


        if(gamepad2.left_bumper == true) {
          SWEEPER_EXT.setPostion(.75);
          SWEEEPER_EXT2.setPostion(.2);
          telemetry.addLine("Seepexte = .9")
          telemetry.update();
          }
        else if (gamepad2.right_bumper == true) {
          SWEEPER_EXT.setPostion(.27);
          SWEEPER_EXT2.setPostion(.68);
          telemetry.addLine("Sweetext = .1");
          telemetry.update();
         }

        */

        }
    }

    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFrontWheel, DcMotor rightFrontWheel, DcMotor leftBackWheel, DcMotor rightBackWheel){

        double leftFrontVal = y - x - c;
        double rightFrontVal = y + x + c;
        double leftBackVal = y + x - c;
        double rightBackVal = y - x + c;

        // Move range to between 0 and +1, if not alreaduy
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, leftFrontVal};
        Arrays.sort(wheelPowers);

        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        double scaledPower = SCALEDPOWER;

        leftFrontWheel.setPower((leftFrontVal * scaledPower  + leftFrontWheel.getPower() * (1-scaledPower) ));   //  90   % power
        rightFrontWheel.setPower((rightFrontVal * scaledPower  + rightFrontWheel.getPower() * (1-scaledPower) ) );//  90   % power
        leftBackWheel.setPower((leftBackVal * scaledPower + leftBackWheel.getPower() * (1-scaledPower)) );            // 100   % power
        rightBackWheel.setPower((rightBackVal * scaledPower + rightBackWheel.getPower() * (1-scaledPower)) );          // 100   % power

    }

}


