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


/*

This is a simple program that will use the REV Robotics BlinkedIN Controller

- We will be using the 5V Addressable LED strips

- YOu need to use the PWM signal from a servo port in order to controll which LED pattern
  you want displayed
 */
@TeleOp(name = "LED Test", group = "CS9977")
//@Autonomous(name = "LED Test", group = "CS9977")
//@Disabled

public class blinkedIN extends LinearOpMode {


    // Declare Hardare Mappings
    //skyHardwareMap robot2 = new skyHardwareMap();
    //ElapsedTime runtime = new ElapsedTime();


    // Define class members
    //ServoImplEx servo;  // We will use this to Controll the Rev BlinkedIN LED controller via PWM Signals
    //PwmControl.PwmRange range = new PwmControl.PwmRange(1005, 1999);


    // This section is for the Rev Extensions 2 Module Testing

    ExpansionHubServo servo;


    @Override
    public void runOpMode() {


        // This section is specific to to BlinkedIN'

        servo = (ExpansionHubServo) hardwareMap.servo.get("lights");    // This is an example using Rev Extensions2 modules

        //servo = hardwareMap.get(ServoImplEx.class, "lights"); // Commented out after incorportating into skyHardwaremap.java file
        //servo.setPwmRange(range);                                        // Commented out after incorportating into skyHardwaremap.java file

        // Wait for the start button

        waitForStart();
        while (opModeIsActive()) {


            // This section contains sample code that is from here:
            // https://github.com/OpenFTC/RevExtensions2/blob/master/examples/src/main/java/org/openftc/revextensions2/examples/ServoPulseWidthExample.java


            servo.setPwmEnable();           // Important:  Make sure the servo is enable, otherwise setPulseWidUs() will not work
            servo.setPulseWidthUs(1795);    // Set to DarkRed = 1795 uS
            sleep(10000);       // set Dark Red to show for 10sec





         /*   Commenting the following sections as a test of the Expansionhun Extensions 2 Module
            // Set to Dark RED
            PwmControl.PwmRange DarkRed = new PwmControl.PwmRange(1795,1795);
            servo.setPwmRange(DarkRed);
            sleep(5000);

            // Set to to Gold
            PwmControl.PwmRange Gold = new PwmControl.PwmRange(1835, 1835);
            servo.setPwmRange(Gold);
            sleep(5000);

            // Set to Dark Green
            PwmControl.PwmRange DarkGreen = new PwmControl.PwmRange(1875, 1875);
            servo.setPwmRange(DarkGreen);
            sleep(5000);

            //Set tp Dark Blue
            PwmControl.PwmRange DarkBlue = new PwmControl.PwmRange(1925, 1925);
            servo.setPwmRange(DarkBlue);
            sleep(5000);


        }


        */
        }
    }
}
