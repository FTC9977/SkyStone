/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareMap.skyHardwareMap;
import org.openftc.revextensions2.ExpansionHubServo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/*

This is a simple program that will use the REV Robotics BlinkedIN Controller

- We will be using the 5V Addressable LED strips

 */
@TeleOp(name = "LED Test", group = "CS9977")

//@Disabled

public class blinkedIN extends LinearOpMode {

     RevBlinkinLedDriver blinkinLedDriver;
     RevBlinkinLedDriver.BlinkinPattern pattern;
     RevBlinkinLedDriver.BlinkinPattern RedPattern;         // Use this Patter to Signal 15 seconds or less exist in Game
     RevBlinkinLedDriver.BlinkinPattern YellowPatter;       // Use this Pattern to Signal Start of End Game (30seconds)
     RevBlinkinLedDriver.BlinkinPattern GreenPattern;       // Use this Pattern to Signal TeleOP Timer is within Delta
     RevBlinkinLedDriver.BlinkinPattern GoldPattern;        // Use this pattern to signal we are only seeing plain stone
     RevBlinkinLedDriver.BlinkinPattern FoundStonePattern;  // Use this to signal we found Stone in AS


 @Override
    public void runOpMode() {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        pattern = RevBlinkinLedDriver.BlinkinPattern.AQUA;





        // Wait for the start button

        waitForStart();
        while (opModeIsActive()) {


            // This section contains sample code that is from here:
            https://github.com/ftctechnh/ftc_app/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/SampleRevBlinkinLedDriver.java


            blinkinLedDriver.setPattern(pattern);
            sleep(5000);

            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);    // Here you can change the pattern to any one of the available color options for the 5V addressable LED strips
            sleep(5000);

        }



        }
    }

