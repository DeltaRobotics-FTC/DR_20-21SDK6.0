package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//other imports

//genneral

//odometry

//tensor flow

//velocity

//auto aim




import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@TeleOp(name = "blueAuto")
public class blueAuto extends LinearOpMode {

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
  //genneral

  //odometry

  //TFOD

  //velocity

  //auto aim


  @Override
  public void runOpMode() throws InterruptedException {
    initVuforia();
    initTfod();

    //init 
    if (tfod != null) {
      tfod.activate();

      // The TensorFlow software will scale the input images from the camera to a lower resolution.
      // This can result in lower detection accuracy at longer distances (> 55cm or 22").
      // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
      // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
      // should be set to the value of the images used to create the TensorFlow Object Detection model
      // (typically 1.78 or 16/9).

      // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
      tfod.setZoom(3, 1.78);
    }
    //genneral

    //odometry

    //TFOD

    //velocity

    //auto aim
    telemetry.addData(">", "Press Play to start op mode");
    telemetry.update();
    waitForStart();

    while (opModeIsActive()) {
      if (opModeIsActive()) {
        while (opModeIsActive()) {
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
        }
      }




      if (tfod != null) {
        tfod.shutdown();
      }



      sleep(3000);

     if(view == "Quad")
     {
      drive.timeDrive(500,1,driveStyle.FORWARD);
     }
     else if(view == "Single")
     {
       drive.timeDrive(500,1,driveStyle.BACKWARD);

     }
     else
      {
        drive.timeDrive(500,1,driveStyle.STRAFE_LEFT);

      }

      //Run Op Mode

      //see rings with tensor flow

      //drive up to line and shoot with auto aim

      //go place wabble goal

      //drive back and pick up starter stack (if 4 shoot first 1-2 now with auto aim)

      //pick up other wable goal

      //grab any remaining rings

      //drive up to line

      //shoot remaining rings with auto aim

      //drop off wable goal

      //drive to line
    }
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

    // Loading trackables is not necessary for the TensorFlow Object Detection engine.

  }

  public void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    tfodParameters.minResultConfidence = 0.6f;
    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
  }
  public blueAuto(LinearOpMode linearOpMode) {
    // Drive class for first meet with mecanum wheels
    drive = new MecanumDriveTrain(linearOpMode);
  }

}




  /*
  public voids
  */
  
  //odometry
  
  //TFOD
  
  //Velocity
  
  //auto aim
  

