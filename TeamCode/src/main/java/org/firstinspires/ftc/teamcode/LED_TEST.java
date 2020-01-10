package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;




import java.util.Arrays;


@TeleOp(name="LED Test_TELOP", group="calebs_robot")
//@Disabled
public class LED_TEST extends LinearOpMode {


    // This is a test OP Mode to test  runtime.seconds() timer and how we
    // are using it to control/set different color patterns using the REV BLinkin Controller

    //  If successfull we will use this code to provide visual clues to driver about certain
    //  sensor states and or critial timers during the game (i.e a signal when there is 10 seconds left in TeleOP... we should be parking...)

  




    // Created Rev Robotics BlinkIN instances



    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    ElapsedTime runtime = new ElapsedTime();





    @Override
    public void runOpMode() throws InterruptedException {




        // Set the Initial; Blinkin Color Schema to Aqua (Represents CS9977 Team Colors

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        blinkinLedDriver.setPattern(pattern);

        waitForStart();

        while (opModeIsActive()) {




            // Adding section for REV Blinkin Controls for Game Timers ONLY......This does NOT inclide LED colors for closing of claw
     /*
            if ((runtime.seconds() >= 90)) {
                //pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
                //blinkinLedDriver.setPattern(pattern);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);

            } else if ((runtime.seconds() >= 100)) {
               // pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
               // blinkinLedDriver.setPattern(pattern);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

            } else if ((runtime.seconds() >= 110)) {
                //pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                //blinkinLedDriver.setPattern(pattern);
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
       */

            if ((runtime.seconds() >= 90) &&  runtime.seconds() <= 99) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            }
            if ((runtime.seconds() >= 100) && runtime.seconds() <= 109) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            }
            if ((runtime.seconds() >= 110) && runtime.seconds() <= 120) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }


            telemetry.addData("Time ", runtime.seconds());
            telemetry.update();

        }
    }


}


