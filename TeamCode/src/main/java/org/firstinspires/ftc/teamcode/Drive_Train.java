package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private static Servo   ArmServo1,LHook,RHook,PurpleArmLeft,GreenArmRight,ArmServo2,claw;
    int A;
    int B;
    int C;
    private static RevBlinkinLedDriver blinkinLedDriver;
    private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static TouchSensor Touch;

    ElapsedTime runtime = new ElapsedTime();

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

        ArmServo1 = hardwareMap.servo.get(UniversalConstants.ArmServo1);
        ArmServo2 = hardwareMap.servo.get(UniversalConstants.ArmServo2);
        LHook = hardwareMap.servo.get(UniversalConstants.LHook);
        RHook = hardwareMap.servo.get(UniversalConstants.RHook);
        PurpleArmLeft = hardwareMap.servo.get(UniversalConstants.PurpleArmLeft);
        GreenArmRight = hardwareMap.servo.get(UniversalConstants.GreenArmRight);
        claw = hardwareMap.servo.get(UniversalConstants.claw);

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;

        Touch = hardwareMap.touchSensor.get( "touch");


        LiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);


       rightBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        InLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE);

        waitForStart();
         A = 0;
         B = 0;
         C = 0;
         runtime.reset();
        while (opModeIsActive()) {

            if (gamepad1.x && (A <= 0)){
               claw.setPosition(.1); // close
                A = A +1;
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }else if (gamepad1.x && (A >= 1)){
                claw.setPosition(.9); // open
                A = A -1;
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }


            if (gamepad1.left_bumper){
                PurpleArmLeft.setPosition(.99);
                //GreenArmRight.setPosition(.1);
            }else {
                PurpleArmLeft.setPosition(.1);
                //GreenArmRight.setPosition(.99);
            }

            if (gamepad1.right_bumper){
                LHook.setPosition(.1);
                RHook.setPosition(.8);
            } else {
                LHook.setPosition(.5);
                RHook.setPosition(.4);
            }

            if (gamepad1.a) {
                 LiftLeft.setPower(.75);
                 LiftRight.setPower(.75);
            } else if (gamepad1.b){
                LiftLeft.setPower(-.5);
                LiftRight.setPower(-.5);
            } else {
                LiftLeft.setPower(.1);
                LiftRight.setPower(.1);
            }


            if (gamepad1.dpad_down){
                InLeft.setPower(.5);
                InRight.setPower(.5);
                B = B + 1;
            }else if (gamepad1.dpad_up){
                InLeft.setPower(-.5);
                InRight.setPower(-.5);
                B = B + 1;
            }else if (gamepad1.dpad_left){
                InLeft.setPower(0);
                InRight.setPower(0);
                B = 0 ;
            }

            if (gamepad1.dpad_up || gamepad1.dpad_down ){
                ArmServo1.setPosition(.7);   // Raise it to accept block
                ArmServo2.setPosition(.3);
            }

            if (gamepad1.right_trigger >= .5){
                ArmServo1.setPosition(.1);
                ArmServo2.setPosition(.9);
            }
            if (gamepad1.left_trigger >= .5 || (C == 1)){
                ArmServo1.setPosition(.1);
                ArmServo2.setPosition(.9);
            }
             if ((gamepad1.right_trigger <= .5) && ( B == 0 ) ) {
                 ArmServo1.setPosition(.9);
                 ArmServo2.setPosition(.1);
             }

             if (Touch.isPressed()){
                 blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
             } else if ((Touch.isPressed()== false) && A <= 0){
                 blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
             }

            if ((runtime.seconds() >= 90) &&  runtime.seconds() <= 99) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            }
            if ((runtime.seconds() >= 100) && runtime.seconds() <= 109) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
            if ((runtime.seconds() >= 110) && runtime.seconds() <= 120) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);
            }

            telemetry.addData("Time ", runtime.seconds());
            telemetry.update();


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


