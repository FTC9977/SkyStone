package org.firstinspires.ftc.teamcode.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Vision.CSvuforiaStuff;


@Disabled           // UNCOMMET OUT to Restore to Phone

@Autonomous(name="BlueMaine Auto", group="csnull")

public class BlueMainAuto extends LinearOpMode {


    // Vuforia Setup & Key
     public CSvuforiaStuff CSvuforiaStuff;
     private VuforiaLocalizer vuforia;
     private static final String VUFORIA_KEY = "AdphoDD/////AAABmfGMegzzNEdbhS/kSfVGvz98llAmaZL0fNYksiN4KmM/VJN0pA8I/dy1zkr644MTRCK15XeOhtBTgGAmWyj+9lPzJ37hekqXsYscF0h1rcwLREkAAc+N2ZGNdbfmVzj+YL60ZW0NjwhH6l/E2io081/784lOsZwBKfIM8u/ouynu9B7PeTo7Z0YUHOe9taFajGxt7WV7G7SKhmM+IHjAEgR4q7NMrf72Rqy11QzKEqEznW8xjWJhRHG5hpL0Rjnmqv47NcXM+GyiFMfArIZEndtGlijTXt5njO8MpVOCLkWnVQAkhHLnAjqa9H6hJTwMcIRMqaE4NeG8jONzlqYZp/dK589uco69+uFIoRDZ9qmU";
     CSvuforiaStuff.skystonePos pos;


    int stoneDiff;


    @Override
    public void runOpMode() throws InterruptedException {


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;   // Change to front if needed
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        CSvuforiaStuff = new CSvuforiaStuff(vuforia);
        pos = CSvuforiaStuff.vuforiascan(false, false);

        switch (pos){
            case LEFT:
               //PIDDriveForward(80,90,24);
               telemetry.addLine ("Skystone is in the LEFT position");
                sleep(10000);
                telemetry.update();
               stoneDiff = 0;
               break;

            case CENTER:
                //PIDDriveStrafeRight(25, 90, 24);
                //PIDDriveForward(80, 90, 24);
                telemetry.addLine("Skystone is in the CENTER position");
                sleep(10000);
                telemetry.update();
                stoneDiff = 16;
                break;

            case RIGHT:
                //PIDDriveStrafeRight(25, 90, 48);
                //PIDDriveForward(80,90,24);
                telemetry.addLine("Skystone is in the RIGHT postion");
                sleep(10000);
                telemetry.update();
                stoneDiff = 20;
                break;

        }

        // Need to Figure out where this takes you....

        if (pos == org.firstinspires.ftc.teamcode.Vision.CSvuforiaStuff.skystonePos.LEFT) {
            //PIDDriveForward(25, 90, 12);
            telemetry.addData("Skystone Posting is: ", pos);
            sleep(10000);
            telemetry.update();
        } else {
            //PIDDriveForward(25, 90, 48);
            telemetry.addData("Other STone Position ", pos);
            sleep(10000);
            telemetry.update();
        }



      //  See for Example Code: https://github.com/deanlange/Skystone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/BlueMainAuto.java
      //
      // grab stone
      // lift stone
      // turn 90 Degress
      // Drive to Foundation
      // Engage Grabbers
      // sleep (1000);
      // Move Foundation
      // Release Stone
      // Drive to bridge

    }   // END of Main OpMode




} // END Brace for Entire Program