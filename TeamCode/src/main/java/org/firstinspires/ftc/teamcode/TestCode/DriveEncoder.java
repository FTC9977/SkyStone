package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;



@Disabled       // UNCOMMENT OUT TO RESTORE to PHONE


@Autonomous(name="Drive Encoder", group="pid-test")


public class DriveEncoder extends LinearOpMode {

    DcMotor testMotor;
    double tickCount = 543.2;    // GoBilda Yellow Jacket 5202 312RPM Motor

 @Override

 public void runOpMode() throws InterruptedException {

     testMotor = hardwareMap.dcMotor.get("LF");

     testMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
     testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

     // Reset Target Position to zero
     testMotor.setTargetPosition(0);
     testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

     telemetry.addData("Mode ", "Waiting");
     telemetry.update();

     waitForStart();

     telemetry.addData("Mode ", " running");
     telemetry.update();

     // Start Motor and run util it reaches a specific tick count (tickCount))

     double MotorMoving = (tickCount);



     // Set Actual Target Position for the number of Encoder Ticks defined above
     testMotor.setTargetPosition((int) MotorMoving);

     // Set Power
     testMotor.setPower(.75);    // Set to 25% Power


     while (opModeIsActive() && testMotor.isBusy()) {
         telemetry.addData("encoder-fw" , testMotor.getCurrentPosition() + "    busy=" + testMotor.isBusy());
         telemetry.addData("SW Defined Tickcount", tickCount);
         telemetry.update();
         idle();

     }

     testMotor.setPower(0.0);


     // Wait 10
     // .seconds so you can observe the final encoder position
     resetStartTime();

     while (opModeIsActive() && getRuntime() < 10) {
         telemetry.addData("Final Encoder Count is: ", testMotor.getCurrentPosition());
         telemetry.update();
         idle();

     }

 }

}
