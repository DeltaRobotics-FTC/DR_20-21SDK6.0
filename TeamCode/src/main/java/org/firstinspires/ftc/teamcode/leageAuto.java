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
@Autonomous(name = "leageAuto")
public class leageAuto extends LinearOpMode
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
    public static int wobbleDown = 825;
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

        Trajectory shotPosA = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(0))
                .build();

        //drive to power shots
        Trajectory WobbleA = drive.trajectoryBuilder(shotPosA.end(), true)
                .splineToConstantHeading(new Vector2d(10, 40), Math.toRadians(0))
                .build();

        Trajectory Wobble2GrabA = drive.trajectoryBuilder(WobbleA.end())
                .lineTo(new Vector2d(-55, 5))
                .build();

        Trajectory Wobble2strafeA = drive.trajectoryBuilder(Wobble2GrabA.end())
                .lineTo(new Vector2d(-56, 23))
                .build();

        Trajectory Wobble2PlaceA = drive.trajectoryBuilder(Wobble2strafeA.end(), true)
                .splineToConstantHeading(new Vector2d(6, 40), Math.toRadians(20))
                .build();

        //park
        Trajectory ParkA = drive.trajectoryBuilder(Wobble2PlaceA.end())
                .lineTo(new Vector2d(12, 35))
                .build();

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.update();


        //B trajectories

        Trajectory shotPosB = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-9, 48), Math.toRadians(0))
                .build();

        //drive to power shots
        Trajectory WobbleB = drive.trajectoryBuilder(shotPosB.end(), true)
                .splineToConstantHeading(new Vector2d(30, 18), Math.toRadians(0))
                .build();

        Trajectory CollectionB = drive.trajectoryBuilder(WobbleB.end())
                .splineToConstantHeading(new Vector2d(-15, 48), Math.toRadians(0))
                .build();

        Trajectory CollectB = drive.trajectoryBuilder(CollectionB.end())
                .lineTo(
                        new Vector2d(-30, 48),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory shotPos2B = drive.trajectoryBuilder(CollectB.end(), true)
                .splineToConstantHeading(new Vector2d(-9, 48), Math.toRadians(0))
                .build();

        //park
        Trajectory ParkB = drive.trajectoryBuilder(shotPos2B.end())
                .lineTo(new Vector2d(12, 25))
                .build();

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.addData("B built", "");
        telemetry.update();


        //C trajectories

        //drive to wobble spot trajectory

        Trajectory shotPosC = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-8, 48), Math.toRadians(0))
                .build();

        //drive to power shots
        Trajectory WobbleC = drive.trajectoryBuilder(shotPosC.end(), true)
                .splineToConstantHeading(new Vector2d(50, 40), Math.toRadians(0))
                .build();

        Trajectory CollectionC = drive.trajectoryBuilder(WobbleC.end())
                .splineToConstantHeading(new Vector2d(0, 40), Math.toRadians(0))
                .build();

        Trajectory CollectC = drive.trajectoryBuilder(CollectionC.end())
                .lineTo(
                        new Vector2d(-30, 40),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory shotPos2C = drive.trajectoryBuilder(CollectC.end(), true)
                .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(0))
                .build();

        //park
        Trajectory ParkC = drive.trajectoryBuilder(shotPos2C.end())
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

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                drive.followTrajectory(shotPosC);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                sleep(shotTiming*2);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(WobbleC);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.servo2.setPosition(wobbleOpen);

                sleep(wobbleServoWait);

                sleep(1000);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);

                drive.followTrajectory(CollectionC);

                drive.followTrajectory(CollectC);

                sleep(350);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                drive.followTrajectory(shotPos2C);

                sleep(shotTiming*2);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(ParkC);

                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);

                break;
            case LABEL_SECOND_ELEMENT:
                //drive to B

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                drive.followTrajectory(shotPosB);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                sleep(shotTiming*2);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(WobbleB);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.servo2.setPosition(wobbleOpen);

                sleep(wobbleServoWait);

                sleep(3000);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);

                drive.followTrajectory(CollectionB);

                drive.followTrajectory(CollectB);

                sleep(350);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                drive.followTrajectory(shotPos2B);

                sleep(shotTiming*2);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(ParkB);

                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);

                break;
            default:
                //drive to A

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                drive.followTrajectory(shotPosA);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                sleep(shotTiming*2);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);
                sleep(shotTiming);

                robot.servo.setPosition(servoShoot);
                sleep(shotTiming);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                drive.followTrajectory(WobbleA);

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

                drive.followTrajectory(Wobble2GrabA);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive.followTrajectory(Wobble2strafeA);

                robot.servo2.setPosition(wobbleClosed);

                sleep(wobbleServoWait);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                drive.followTrajectory(Wobble2PlaceA);

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

                while (robot.wobble.isBusy())
                {
                }

                sleep(1000);

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
