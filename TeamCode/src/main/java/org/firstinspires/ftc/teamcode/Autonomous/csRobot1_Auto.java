package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareMap.skyHardwareMap;

public class csRobot1_Auto {
    private LinearOpMode autonomous; // The autnomouse class being run
    private skyHardwareMap hardware;
    private long startTime;

    public void setStartTime(long time) {
        startTime = System.currentTimeMillis();


    }

}
