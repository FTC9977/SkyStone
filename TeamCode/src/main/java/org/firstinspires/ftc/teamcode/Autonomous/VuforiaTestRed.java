package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.HardwareMap.Robot;




@Disabled       //UNCOMMENT OUT to restore to Phone



@Autonomous (name = "Vuforia Test R", group = "csnull")
public class VuforiaTestRed extends Robot {

    VuforiaStuff.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();
        pos = vuforiaStuff.vuforiascan(true, true );
        telemetry.addData("Pos: ", pos);

        telemetry.update();
        sleep(3000);
    }
}