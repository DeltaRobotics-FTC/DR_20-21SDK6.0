package org.firstinspires.ftc.teamcode;

//imports


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;
import java.util.List;

@Config
@Autonomous(name = "Meet5Auto")
public class Meet5Auto extends LinearOpMode
{

//declare variables

    //Viewforia / TFOD variables
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AQIjJXP/////AAABmX8DXrmUxEBjvVNbT94EWcg3A75NZTjC3HG9/ur6NlOGrwrPUBWwLK8GlSeDl/fPcBsf+HkwYZQt7Fu8g/fJSvgftOYprWUaAWTCcyEnjfqU7CKCEEeWOO97PEJHdsjSPaRCoKAUjmRCknWJWxPuvgBXU4z63zwtr45AR0DzsF9FRdoj9pNR7hcmPKZmMLSfU6zdeBinzk2DQrJq2GGHJJgI0Mgh/IcrRA54NaGttRaqLpvLOuDHRiPyHnOtOXkjHBZp4Simdyqht675alc36Kyz3PF34/9X6m3b/43kuI231AaSBt1r5GnQv0jL9QRbGde2lr0U8mTmnatRm1ASpgCIcAJJ82jRpyWf3yELRH1w";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public String view = "";


    //wobble goal variables
    public static int wobbleUp = 425;
    public static int wobbleDown = 925;
    public static int wobbleAway = 0;

    public static double wobbleArmPower = 0.7;

    public static double wobbleOpen = 1;
    public static double wobbleClosed = 0;

    public static int wobbleServoWait = 250;


    //shooter
    public static double flywheelHighSpeed = 1700;
    public static double flywheelPowerSpeed = 1550;

    public static double servoShoot = -.1;
    public static double servoRetract = .25;

    public static int shotTiming = 750;


    //RR pose
//Pose2d _ = new Pose2d(_, _, Math.toRadians(_));
//Vector2d _ = new Vector2d(_, _);
    public static Pose2d startPose = new Pose2d(-63, 57, Math.toRadians(180));


    @Override
    public void runOpMode() throws InterruptedException
    {

        //classes

        //dashboard init
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        //robot hardware map init
        RobotHardware robot = new RobotHardware(hardwareMap);

        //roadrunner drive constants init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("classes inited", "");
        telemetry.update();


        //cammera

        //veiwforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //TFOD
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;                                                     //change to change mimimum confidence
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(3, 1.78);
        }

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.update();


        //other init stuff
        robot.wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobble.setTargetPosition(wobbleAway);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.servo2.setPosition(wobbleClosed);

        robot.servo.setPosition(servoRetract);

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.update();


        //with roadrunner you basicly build auto in init then run the built auto

        //set start position
        drive.setPoseEstimate(startPose);


        //A trajectories 

        //drive to wobble spot A
        Trajectory Wobble1DeliveryA = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(10, 48), Math.toRadians(0))
                .build();

        //drive to power shots
        Trajectory PowerShot1A = drive.trajectoryBuilder(Wobble1DeliveryA.end())
                .splineToConstantHeading(new Vector2d(-5, 48), Math.toRadians(180))
                .build();

        Trajectory PowerShot2A = drive.trajectoryBuilder(PowerShot1A.end())
                .strafeLeft(0.01)
                .build();

        Trajectory PowerShot3A = drive.trajectoryBuilder(PowerShot2A.end())
                .strafeLeft(0.01)
                .build();

        //park
        Trajectory ParkA = drive.trajectoryBuilder(PowerShot3A.end())
                .lineTo(new Vector2d(12, 35))
                .build();

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.update();


        //B trajectories

        //drive to wobble spot B
        Trajectory Wobble1DeliveryB = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(30, 24), Math.toRadians(0))
                .build();

        //drive to power shots
        Trajectory PowerShot1B = drive.trajectoryBuilder(Wobble1DeliveryB.end())
                .splineTo(new Vector2d(-11, 53), Math.toRadians(182.5))
                .build();

        Trajectory PowerShot2B = drive.trajectoryBuilder(PowerShot1B.end())
                .strafeLeft(0.01)
                .build();

        Trajectory PowerShot3B = drive.trajectoryBuilder(PowerShot2B.end())
                .strafeLeft(0.01)
                .build();

        //park
        Trajectory ParkB = drive.trajectoryBuilder(PowerShot3B.end())
                .lineTo(new Vector2d(12, 35))
                .build();

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.addData("B built", "");
        telemetry.update();


        //C trajectories

        //drive to wobble spot trajectory
        Trajectory Wobble1DeliveryC = drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(50, 48), Math.toRadians(0))
                .build();

        //drive to power shots
        Trajectory PowerShot1C = drive.trajectoryBuilder(Wobble1DeliveryC.end())
                .splineToConstantHeading(new Vector2d(-11, 48), Math.toRadians(180))
                .build();

        Trajectory PowerShot2C = drive.trajectoryBuilder(PowerShot1C.end())
                .strafeLeft(0.01)
                .build();

        Trajectory PowerShot3C = drive.trajectoryBuilder(PowerShot2C.end())
                .strafeLeft(0.01)
                .build();

        //park
        Trajectory ParkC = drive.trajectoryBuilder(PowerShot3C.end())
                .lineTo(new Vector2d(12, 35))
                .build();

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.addData("B built", "");
        telemetry.addData("C built", "");
        telemetry.update();


        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.addData("B built", "");
        telemetry.addData("C built", "");
        telemetry.update();

        waitForStart();

        //auto

        //get TFOD detection
        if (tfod != null)
        {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions)
                {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    view = recognition.getLabel();
                }
                telemetry.update();
                if (tfod != null)
                {
                    tfod.shutdown();
                }
            }
        }

        //
        // switch statement for 0,1,4 rings
        //


        switch (view)
        {
            case LABEL_FIRST_ELEMENT:
                //drive to C 

                drive.followTrajectory(Wobble1DeliveryC);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.servo2.setPosition(wobbleOpen);

                sleep(wobbleServoWait);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                drive.followTrajectory(PowerShot1C);

                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                drive.followTrajectory(PowerShot2C);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                drive.followTrajectory(PowerShot3C);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(ParkC);

                break;
            case LABEL_SECOND_ELEMENT:
                //drive to B

                drive.followTrajectory(Wobble1DeliveryB);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.servo2.setPosition(wobbleOpen);

                sleep(wobbleServoWait);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                sleep(wobbleServoWait);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                drive.followTrajectory(PowerShot1B);

                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                drive.followTrajectory(PowerShot2B);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                drive.followTrajectory(PowerShot3B);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(ParkB);

                break;
            default:
                //drive to A

                drive.followTrajectory(Wobble1DeliveryA);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.servo2.setPosition(wobbleOpen);

                sleep(wobbleServoWait);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                drive.followTrajectory(PowerShot1A);

                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                drive.followTrajectory(PowerShot2A);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                drive.followTrajectory(PowerShot3A);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(ParkA);

        }
    }
}

// auto list

//~take pic: ~tensor flow, ~no variables

//drive to floor goal: ~roadrunner, goal pose variable

//drop wobble1: ~wobble arm, ~servo, ~up and down variables, ~open closed variables, ~time for servo variable

//shoot power shots: ~roadrunner, first shot pose variable, strafe distance variable, PIDF (HWmap?) and ~flywheel variables, ~shoot and retract servo variables, ~how long between shots variable

//grab wobble2: ~roadrunner, final pose variable, ~up and down variables, ~open closed variables, ~time for servo variable

//deliver wobble2: ~roadrunner, wobble1 offset variable

//shoot starter stack: ~roadrunner, pose varables plural, intake speed variable, drive speed variable, PIDF and ~flywheel variables, ~shoot and retract servo variables, ~how long between shots

//park: ~roadrunner, pose variable

//list remaining : RRPose variables, intake variables and intake
