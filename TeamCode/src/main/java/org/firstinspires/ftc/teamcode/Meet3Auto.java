package org.firstinspires.ftc.teamcode;

//imports


@Config
@Autonomous(name = "Meet3Auto")
public class Meet3Auto extends LinearOpMode {

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
public static int wobbleUp = -425;
public static int wobbleDown = -925;
public static int wobbleAway = 0;
    
public static double wobbleArmPower = 0.7;
    
public static double wobbleOpen = 1;
public static double wobbleClosed = 0;
    
public static double wobbleServoWait = 250;



//shooter
public static double flywheelHighSpeed = 1700;
public static double flywheelPowerSpeed = 1550;

public static double servoShoot = -.1;
public static double servoRetract = .25;
    
public static double shotTiming = 250;
    
    
    
//RR pose 
//Pose2d _ = new Pose2d(_, _, Math.toRadians(_));
//Vector2d _ = new Vector2d(_, _);
public static Pose2d startPose = new Pose2d(-63, 57, Math.toRadians(180));



    @Override
    public void runOpMode() throws InterruptedException {
    
        //classes
    
        //dashboard init
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        
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
       
       if (tfod != null) {
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
            .splineTo(new Vector2d(10,53), Math.toRadians(180))
            .build();
        
        //drive to power shots
        Trajectory PowerShot1A = drive.trajectoryBuilder(Wobble1DeliveryA.end())
            .splineToConstantHeading(new Vector2d(-9,24), Math.toRadians(180))
            .build();
        
        Trajectory PowerShot2A = drive.trajectoryBuilder(PowerShot1A.end())
            .strafeLeft(7.5)
            .build();
        
        Trajectory PowerShot3A = drive.trajectoryBuilder(PowerShot2A.end())
            .strafeLeft(7.5)
            .build();
        
        //drive to grab second wobble goal
        Trajectory Wobble2GrabA = drive.trajectoryBuilder(PowerShot3A.end())
            .splineTo(new Vector2d(-54,0), Math.toRadians(180))
            .lineToConstantHeading(new Vector2d(-54,9))
            .build();
            
        //drive to wobble2 spot
        Trajectory Wobble2DeliveryA = drive.trajectoryBuilder(Wobble2GrabA.end(), true)
            .splineTo(new Vector2d(2,45), Math.toRadians(180))
            .build();
        
        //park
        Trajectory ParkA = drive.trajectoryBuilder(Wobble2DeliveryA.end())
            .lineTo(new Vector2d(0,35))
            .build();
        
        telemetry.addData("classes inited", "");
        telemetry.addData("cammera inited", "");
        telemetry.addData("building trajectories", "");
        telemetry.addData("A built", "");
        telemetry.update();
            
        
        
        //B trajectories
        
        //drive to wobble spot B
        Trajectory Wobble1DeliveryB = drive.trajectoryBuilder(startPose, true)
            .splineTo(new Vector2d(34,29), Math.toRadians(180))
            .build();
        
        //drive to power shots
        Trajectory PowerShot1B = drive.trajectoryBuilder(Wobble1DeliveryB.end())
            .splineToConstantHeading(new Vector2d(-9,24), Math.toRadians(180))
            .build();
        
        Trajectory PowerShot2B = drive.trajectoryBuilder(PowerShot1B.end())
            .strafeLeft(7.5)
            .build();
        
        Trajectory PowerShot3B = drive.trajectoryBuilder(PowerShot2B.end())
            .strafeLeft(7.5)
            .build();
        
        //drive to grab second wobble goal
        Trajectory Wobble2GrabB = drive.trajectoryBuilder(PowerShot3B.end())
            .splineTo(new Vector2d(-54,0), Math.toRadians(180))
            .lineToConstantHeading(new Vector2d(-54,9))
            .build();
            
        //drive to wobble2 spot
        Trajectory Wobble2DeliveryB = drive.trajectoryBuilder(Wobble2GrabB.end(), true)
            .splineTo(new Vector2d(26,21), Math.toRadians(180))
            .build();
        
        //collect some or all of the starter stack
        Trajectory StarterStack1B = drive.trajectoryBuilder(Wobble2DeliveryB.end())
            .splineTo(new Vector2d(-9,36), Math.toRadians(180))
            .build();
        
        //drive slow to intake
        Trajectory Intake1B = drive.trajectoryBuilder(StarterStack1B.end())
            .forward(
                10, 
                new MinVelocityConstraint(
                    Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                    )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();
        
        //shoot what was collected
        Trajectory ShootSpotB = drive.trajectoryBuilder(Intake1B.end(), true)
            .splineToConstantHeading(new Vector2d(-9,55), Math.toRadians(180))
            .build();
        
        //park
        Trajectory ParkB = drive.trajectoryBuilder(ShootSpotB.end(), true)
            .splineToConstantHeading(new Vector2d(0,55), Math.toRadians(180))
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
            .splineTo(new Vector2d(59,53), Math.toRadians(180))
            .build();
        
        //drive to power shots
        Trajectory PowerShot1C = drive.trajectoryBuilder(Wobble1DeliveryC.end())
            .splineToConstantHeading(new Vector2d(-9,24), Math.toRadians(180))
            .build();
        
        Trajectory PowerShot2C = drive.trajectoryBuilder(PowerShot1C.end())
            .strafeLeft(7.5)
            .build();
        
        Trajectory PowerShot3C = drive.trajectoryBuilder(PowerShot2C.end())
            .strafeLeft(7.5)
            .build();
        
        //drive to grab second wobble goal
        Trajectory Wobble2GrabC = drive.trajectoryBuilder(PowerShot3C.end())
            .splineTo(new Vector2d(-54,0), Math.toRadians(180))
            .lineToConstantHeading(new Vector2d(-54,9))
            .build();
            
        //drive to wobble2 spot
        Trajectory Wobble2DeliveryC = drive.trajectoryBuilder(Wobble2GrabC.end(), true)
            .splineTo(new Vector2d(51,45), Math.toRadians(180))
            .build();
        
        //collect some or all of the starter stack
        Trajectory StarterStack1C = drive.trajectoryBuilder(Wobble2DeliveryC.end())
            .splineTo(new Vector2d(-9,36), Math.toRadians(180))
            .build();
        
        //drive slow to intake
        Trajectory Intake1C = drive.trajectoryBuilder(StarterStack1C.end())
            .forward(
                10, 
                new MinVelocityConstraint(
                    Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                    )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();
        
        //shoot what was collected
        Trajectory ShootSpotC = drive.trajectoryBuilder(Intake1C.end(), true)
            .splineToConstantHeading(new Vector2d(-9,55), Math.toRadians(180))
            .build();
        
        //go back for more if C
        Trajectory StarterStack2C = drive.trajectoryBuilder(ShootSpotC.end())
            .splineToConstantHeading(new Vector2d(-9,36), Math.toRadians(180))
            .build();
            
        //drive slow to intake
        Trajectory Intake2C = drive.trajectoryBuilder(StarterStack2C.end())
            .forward(
                10, 
                new MinVelocityConstraint(
                    Arrays.asList(
                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                    )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
            )
            .build();
        
        //shoot remaining rings
        Trajectory ShootSpot2C = drive.trajectoryBuilder(Intake2C.end(), true)
            .splineToConstantHeading(new Vector2d(-9,55), Math.toRadians(180))
            .build();
        
        //park
        Trajectory ParkC = drive.trajectoryBuilder(ShootSpot2C.end(), true)
            .splineToConstantHeading(new Vector2d(0,55), Math.toRadians(180))
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
        
        
        
        
        
        
        
        
        
        
        
        
        //A
        
        drive.followTrajectory(Wobble1DeliveryA);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while (robot.wobble.isBusy()) {}
        
        robot.servo2.setPosition(wobbleOpen);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleUp);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelPowerSpeed);
        
        drive.followTrajectory(PowerShot1A);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        drive.followTrajectory(PowerShot2A);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        drive.followTrajectory(PowerShot3A);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        ((DcMotorEx) robot.flywheel).setVelocity(0);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(Wobble2GrabA);
        
        robot.servo2.setPosition(wobbleClosed);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleUp);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(Wobble2DeliveryA);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        robot.servo2.setPosition(wobbleOpen);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleAway);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(ParkA);
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        //B
        drive.followTrajectory(Wobble1DeliveryB);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while (robot.wobble.isBusy()) {}
        
        robot.servo2.setPosition(wobbleOpen);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleUp);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelPowerSpeed);
        
        drive.followTrajectory(PowerShot1B);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        drive.followTrajectory(PowerShot2B);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        drive.followTrajectory(PowerShot3B);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        ((DcMotorEx) robot.flywheel).setVelocity(0);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(Wobble2GrabB);
        
        robot.servo2.setPosition(wobbleClosed);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleUp);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(Wobble2DeliveryB);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        robot.servo2.setPosition(wobbleOpen);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleAway);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(StarterStack1B);
        
        // turn on intake
        
        drive.followTrajectory(Intake1B);
        
        // turn off intake
        
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);
        
        drive.followTrajectory(ShootSpotB);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        sleep(shotTiming);
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        ((DcMotorEx) robot.flywheel).setVelocity(0);
        
        drive.followTrajectory(ParkB);
        
        
        
        
        
        
        
        
        
        
        
        // C
        drive.followTrajectory(Wobble1DeliveryC);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while (robot.wobble.isBusy()) {}
        
        robot.servo2.setPosition(wobbleOpen);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleUp);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelPowerSpeed);
        
        drive.followTrajectory(PowerShot1C);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        drive.followTrajectory(PowerShot2C);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        drive.followTrajectory(PowerShot3C);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        ((DcMotorEx) robot.flywheel).setVelocity(0);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(Wobble2GrabC);
        
        robot.servo2.setPosition(wobbleClosed);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleUp);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(Wobble2DeliveryC);
        
        robot.wobble.setTargetPosition(wobbleDown);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        robot.servo2.setPosition(wobbleOpen);
        
        sleep(wobbleServoWait);
        
        robot.wobble.setTargetPosition(wobbleAway);
        robot.wobble.setPower(wobbleArmPower);
        robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        drive.followTrajectory(StarterStack1C);
        
        // turn on intake
        
        drive.followTrajectory(Intake1C);
        
        // turn off intake
        
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelHighSpeed);
        
        drive.followTrajectory(ShootSpotC);
        
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
        
        drive.followTrajectory(StarterStack2C);
        
        // turn on intake
        
        drive.followTrajectory(Intake2C);
        
        // turn off intake
        
        drive.followTrajectory(ShootSpotC);
        
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        sleep(shotTiming);
        robot.servo.setPosition(servoShoot);
        sleep(shotTiming);
        robot.servo.setPosition(servoRetract);
        
        ((DcMotorEx) robot.flywheel).setVelocity(0);
        
        drive.followTrajectory(ParkC);
        
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

