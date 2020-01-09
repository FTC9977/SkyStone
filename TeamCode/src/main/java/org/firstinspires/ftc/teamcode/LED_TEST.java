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

        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
        blinkinLedDriver.setPattern(pattern);

        waitForStart();

        while (opModeIsActive()) {




            // Adding section for REV Blinkin Controls for Game Timers ONLY......This does NOT inclide LED colors for closing of claw

            if ((runtime.seconds() == 150)) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE;
                blinkinLedDriver.setPattern(pattern);
                sleep(3000);
            } else if ((runtime.seconds() == 160)) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
                blinkinLedDriver.setPattern(pattern);
                sleep(3000);
            } else if ((runtime.seconds() == 170)) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(pattern);
            } else
                pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;
                blinkinLedDriver.setPattern(pattern);



        }
    }


}


