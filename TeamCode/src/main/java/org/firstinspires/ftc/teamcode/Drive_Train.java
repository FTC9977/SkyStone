package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.UniversalConstants;
import java.util.Arrays;


@TeleOp(name="DriveTrain", group="calebs_robot")
//@Disabled
public class Drive_Train extends LinearOpMode {
    /*
    *  Thing to fix
    *       
    *
    *       1. fix the strafing on the back two wheels
    *       2. add command to use the arm
    *       3. chang the shafting on the arm
    *       4. lower the sen of the controller
    *
    *
    * */

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double ACCEPTINPUTTHRESHOLD = .15;
    private static final double SCALEDPOWER = 1; // The emphasis is in the current controller reading (vs. current motor power) on the drive train

    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT1NAME); // "LF";   // Left Front, LF l1     port 2
        leftBackWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT2NAME); // "LR";   // Left Rear, LR, l2      port 3
        rightFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHT1NAME); // "RF";  // Right Front, RF, r1  port 1
        rightBackWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHTNAME2); // "RR";  // Right Rear, RR, r2    port 0

        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {
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

        leftFrontWheel.setPower(leftFrontVal * scaledPower + leftFrontWheel.getPower() * (1-scaledPower));
        rightFrontWheel.setPower(rightFrontVal * scaledPower + rightFrontWheel.getPower() * (1-scaledPower));
        leftBackWheel.setPower(leftBackVal * scaledPower + leftBackWheel.getPower() * (1-scaledPower));
        rightBackWheel.setPower(rightBackVal * scaledPower + rightBackWheel.getPower() * (1-scaledPower));

    }

}


