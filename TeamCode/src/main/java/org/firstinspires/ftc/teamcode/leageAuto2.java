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
@Autonomous(name = "leageAuto2")
public class leageAuto2 extends LinearOpMode
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

    public static int wobbleServoWait = 750;


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

        //shoot rings

        Trajectory shotPosA = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(0))
                .build();

        //place wobble goal
        Trajectory WobbleA = drive.trajectoryBuilder(shotPosA.end(), true)
                .splineToConstantHeading(new Vector2d(10, 40), Math.toRadians(0))
                .build();

        //grab second wobble goal
        Trajectory Wobble2GrabA = drive.trajectoryBuilder(WobbleA.end())
                .lineTo(new Vector2d(-55, 5))
                .build();

        Trajectory Wobble2strafeA = drive.trajectoryBuilder(Wobble2GrabA.end())
                .lineTo(new Vector2d(-54.5, 23))
                .build();
        
        //place second wobble goal
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

        //shoot
        Trajectory shotPosB = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(-10, 48), Math.toRadians(0))
                .build();

        //deliver wobble goals
        Trajectory WobbleB = drive.trajectoryBuilder(shotPosB.end(), true)
                .splineToConstantHeading(new Vector2d(30, 17), Math.toRadians(0))
                .build();

        //collect starter stack
        Trajectory CollectionB = drive.trajectoryBuilder(WobbleB.end())
                .lineTo(new Vector2d(-30, 45))
                .build();

        //grab second wobble goal
        Trajectory Wobble2GrabB = drive.trajectoryBuilder(CollectionB.end().plus(new Pose2d(0, 0, Math.toRadians(10))))
                .lineTo(new Vector2d(-40, 5))
                .build();

        Trajectory Wobble2strafeB = drive.trajectoryBuilder(Wobble2GrabB.end())
                .lineTo(new Vector2d(-60, 17))
                .build();

        //place second wobble goal
        Trajectory Wobble2PlaceB = drive.trajectoryBuilder(Wobble2strafeB.end(), true)
                .lineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)))
                .build();

        //park
        Trajectory ParkB = drive.trajectoryBuilder(Wobble2PlaceB.end())
                .lineTo(new Vector2d(12, 10))
                .build();

        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.addData("B built", "");
        telemetry.update();


        //C trajectories
        
        //deliver wobble goal
        Trajectory WobbleC = drive.trajectoryBuilder(startPose, true)
                .splineToConstantHeading(new Vector2d(50, 40), Math.toRadians(0))
                .build();
        
        //shoot pre loaded rings
        Trajectory ShootC = drive.trajectoryBuilder(WobbleC.end())
                .splineTo(new Vector2d(-12, 42), Math.toRadians(187))
                .build();
        
        //collect 1-2 more and shoot
        Trajectory Collect1C = drive.trajectoryBuilder(ShootC.end())
                .lineToConstantHeading(
                        new Vector2d(-25, 46),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
                
        //collect remaining rings and shoot
        Trajectory Collect2C = drive.trajectoryBuilder(Collect1C.end())
                .lineToConstantHeading(
                        new Vector2d(-35, 46),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        
        //park
        Trajectory ParkC = drive.trajectoryBuilder(Collect2C.end())
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

                drive.followTrajectory(WobbleC);
                
                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.servo2.setPosition(wobbleOpen);

                sleep(wobbleServoWait);

                sleep(150);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }
                
                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);
                
                drive.followTrajectory(ShootC);

                sleep(250);
                
                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                sleep(500);

                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                sleep(500);

                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                
                
                robot.intake1.setPower(1);
                robot.intake2.setPower(1);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);
                
                drive.followTrajectory(Collect1C);

                sleep(1000);
                
                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                sleep(1000);

                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);
                
                drive.followTrajectory(Collect2C);

                sleep(1500);
                
                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                sleep(750);
                
                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                sleep(1250);

                robot.servo.setPosition(servoShoot);
                sleep(750);
                robot.servo.setPosition(servoRetract);
                
                drive.followTrajectory(ParkC);
                
                
                ((DcMotorEx) robot.flywheel).setVelocity(0);

                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
              
              
                break;
            case LABEL_SECOND_ELEMENT:
                //drive to B

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);

                drive.followTrajectory(shotPosB);

                sleep(500);

                robot.servo.setPosition(servoShoot);
                sleep(500);
                robot.servo.setPosition(servoRetract);
                sleep(500);

                robot.servo.setPosition(servoShoot);
                sleep(500);
                robot.servo.setPosition(servoRetract);
                sleep(500);

                robot.servo.setPosition(servoShoot);
                sleep(500);
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

                sleep(250);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (robot.wobble.isBusy())
                {
                }

                robot.intake1.setPower(1);
                robot.intake2.setPower(1);

                ((DcMotorEx) robot.flywheel).setVelocity(1700);

                drive.followTrajectory(CollectionB);

                drive.turn(Math.toRadians(10));

                robot.servo.setPosition(servoShoot);
                sleep(500);
                robot.servo.setPosition(servoRetract);

                ((DcMotorEx) robot.flywheel).setVelocity(0);

                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);

                robot.wobble.setTargetPosition(wobbleDown);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive.followTrajectory(Wobble2GrabB);

                drive.followTrajectory(Wobble2strafeB);

                robot.servo2.setPosition(wobbleClosed);

                sleep(wobbleServoWait);

                robot.wobble.setTargetPosition(wobbleUp);
                robot.wobble.setPower(wobbleArmPower);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive.followTrajectory(Wobble2PlaceB);

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

                sleep(150);

                drive.followTrajectory(ParkB);

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

                sleep(2000);

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

        PositionTransfer.currentPose = drive.getPoseEstimate();
    }
}

// auto list

//shoot power shots: ~roadrunner, first shot pose variable, strafe distance variable, PIDF (HWmap?) and ~flywheel variables, ~shoot and retract servo variables, ~how long between shots variable

//shoot starter stack: ~roadrunner, pose varables plural, intake speed variable, drive speed variable, PIDF and ~flywheel variables, ~shoot and retract servo variables, ~how long between shots
