package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Meet4Auto")
public class Meet4Auto extends LinearOpMode
{

    int shootingSpot = -3200;
    int park = -3750;
    double flywheelSpeed = 1700;
    double shoot = .115;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public String view = "";
    public MecanumDriveTrain drive;
    //init variables
    private static final String VUFORIA_KEY =
            "AQIjJXP/////AAABmX8DXrmUxEBjvVNbT94EWcg3A75NZTjC3HG9/ur6NlOGrwrPUBWwLK8GlSeDl/fPcBsf+HkwYZQt7Fu8g/fJSvgftOYprWUaAWTCcyEnjfqU7CKCEEeWOO97PEJHdsjSPaRCoKAUjmRCknWJWxPuvgBXU4z63zwtr45AR0DzsF9FRdoj9pNR7hcmPKZmMLSfU6zdeBinzk2DQrJq2GGHJJgI0Mgh/IcrRA54NaGttRaqLpvLOuDHRiPyHnOtOXkjHBZp4Simdyqht675alc36Kyz3PF34/9X6m3b/43kuI231AaSBt1r5GnQv0jL9QRbGde2lr0U8mTmnatRm1ASpgCIcAJJ82jRpyWf3yELRH1w";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    public BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initVuforia();
        initTfod();

        RobotHardware robot = new RobotHardware(hardwareMap);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2, 1.78);
        }

        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData(">", "Press Play to start op mode");
        //telemetry.addData("bla", robot.motorRB.getCurrentPosition();
        //telemetry.addData(robot.motorRF.getCurrentPosition());
        //telemetry.addData(robot.motorLF.getCurrentPosition());
        //telemetry.addData(robot.motorLB.getCurrentPosition());
        telemetry.update();



        waitForStart();
        
        
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    view = recognition.getLabel();
                }
                telemetry.update();
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }


        //drive to shooting location
        robot.motorLB.setTargetPosition(shootingSpot);
        robot.motorLF.setTargetPosition(shootingSpot);
        robot.motorRB.setTargetPosition(shootingSpot);
        robot.motorRF.setTargetPosition(shootingSpot);


        robot.motorLB.setPower(0.4);
        robot.motorLF.setPower(0.4);
        robot.motorRB.setPower(0.4);
        robot.motorRF.setPower(0.4);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        while (robot.motorLB.isBusy()) {}

        sleep(200);

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

        //start flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelSpeed);

        while (((DcMotorEx) robot.flywheel).getVelocity() != 1700) {}

        sleep(300);

        //shoot rings
        robot.servo.setPosition(shoot);
        sleep(350);

        robot.servo.setPosition(0.2);
        sleep(350);

        robot.servo.setPosition(shoot);
        sleep(350);

        robot.servo.setPosition(0.2);
        sleep(350);

        robot.servo.setPosition(shoot);
        sleep(350);

        robot.servo.setPosition(0.2);

        //stop flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(0);

        switch (view){
            case LABEL_FIRST_ELEMENT:
                //drive to C goal location
                robot.motorLB.setTargetPosition(shootingSpot*2 + 000);
                robot.motorLF.setTargetPosition(shootingSpot*2 + 300);
                robot.motorRB.setTargetPosition(shootingSpot*2 + 300);
                robot.motorRF.setTargetPosition(shootingSpot*2 + 000);


                robot.motorLB.setPower(0.4);
                robot.motorLF.setPower(0.4);
                robot.motorRB.setPower(0.4);
                robot.motorRF.setPower(0.4);

                robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                while (robot.motorLB.isBusy()) {}

                sleep(200);

                robot.motorLB.setPower(0);
                robot.motorLF.setPower(0);
                robot.motorRB.setPower(0);
                robot.motorRF.setPower(0);
                
                
                robot.wobble.setTargetPosition(-850);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                robot.servo2.setPosition(1);
                
                robot.wobble.setTargetPosition(-425);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                break;
            case LABEL_SECOND_ELEMENT:
                //drive to B goal location
                robot.motorLB.setTargetPosition(shootingSpot - 1000);
                robot.motorLF.setTargetPosition(shootingSpot - 1500);
                robot.motorRB.setTargetPosition(shootingSpot - 1500);
                robot.motorRF.setTargetPosition(shootingSpot - 1000);


                robot.motorLB.setPower(0.4);
                robot.motorLF.setPower(0.4);
                robot.motorRB.setPower(0.4);
                robot.motorRF.setPower(0.4);

                robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);



                while (robot.motorLB.isBusy()) {}

                sleep(200);

                robot.motorLB.setPower(0);
                robot.motorLF.setPower(0);
                robot.motorRB.setPower(0);
                robot.motorRF.setPower(0);
                
                
                robot.wobble.setTargetPosition(-850);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                robot.servo2.setPosition(1);
                
                robot.wobble.setTargetPosition(-425);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                break;
            default:
                //run last option TODO
        }

        //drive to the line
        robot.motorLB.setTargetPosition(park);
        robot.motorLF.setTargetPosition(park);
        robot.motorRB.setTargetPosition(park);
        robot.motorRF.setTargetPosition(park);

        robot.motorLB.setPower(0.4);
        robot.motorLF.setPower(0.4);
        robot.motorRB.setPower(0.4);
        robot.motorRF.setPower(0.4);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the               TensorFlow Object Detection engine.

    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
