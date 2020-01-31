package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuffRED;


abstract public class RobotWebcamRED extends LinearOpMode {

    public VuforiaStuffRED vuforiaStuffRED;
    private VuforiaLocalizer vuforia;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "AdphoDD/////AAABmfGMegzzNEdbhS/kSfVGvz98llAmaZL0fNYksiN4KmM/VJN0pA8I/dy1zkr644MTRCK15XeOhtBTgGAmWyj+9lPzJ37hekqXsYscF0h1rcwLREkAAc+N2ZGNdbfmVzj+YL60ZW0NjwhH6l/E2io081/784lOsZwBKfIM8u/ouynu9B7PeTo7Z0YUHOe9taFajGxt7WV7G7SKhmM+IHjAEgR4q7NMrf72Rqy11QzKEqEznW8xjWJhRHG5hpL0Rjnmqv47NcXM+GyiFMfArIZEndtGlijTXt5njO8MpVOCLkWnVQAkhHLnAjqa9H6hJTwMcIRMqaE4NeG8jONzlqYZp/dK589uco69+uFIoRDZ9qmU";




    public void robotInit() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;   // Change to front if needed
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);



        vuforiaStuffRED = new VuforiaStuffRED(vuforia);


        // This chunk of code gets around the Motorola E4 Disconnect bug.  Should be fixed in SDK 5.3, but adding it as a "backup - JUST IN CASE!!!"
        //

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }


        waitForStart();



        }


    }


