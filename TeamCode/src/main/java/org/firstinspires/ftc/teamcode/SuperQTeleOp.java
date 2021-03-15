package org.firstinspires.ftc.teamcode;

//drive
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//flywheel
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//wobble

//intake



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="SuperQTeleOp" ,group = "")
public class SuperQTeleOp extends LinearOpMode
{

    //drive
    double speed = 1.0;
    double zScale = 0.8;
    double turnSpeed = 1.0;
    double driveSpeed = 1.0;

    //flywheel
    double waitTime = 1000;
    double wheelSpeed = 0;
    double servoPosition = 0.22;

    public static double F = 13.32; // = 32767 / maxV      (do not edit from this number)
    public static double P = 20; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double I = 0;// = 0.1 * P           (fine ajustment of P)
    public static double D = 1; // = 0                     (raise to reduce ocolation)

    //wobble
    int upPosition = 300;
    int grabPosition = 875;
    double openPosition = 1;
    double closedPosition = 0;


    //intake
    double intakeSpeed = .75;
    
    
    //toggles
    boolean leftStickToggle1 = true;
    boolean leftStickToggle2 = false;
    
    boolean rightStickToggle1 = true;
    boolean rightStickToggle2 = false;
    
    boolean yToggle1 = true;
    boolean yToggle2 = false;
    
    boolean xToggle1 = true;
    boolean xToggle2 = false;
    
    boolean aToggle1 = true;
    boolean aToggle2 = false;
    
    boolean lBumperToggle1 = true;
    boolean lBumperToggle2 = false;
    
    boolean dPadUpToggle1 = true;
    boolean dPadUpToggle2 = false;
    
    boolean dPadDownToggle1 = true;
    boolean dPadDownToggle2 = false;
    
    int wobbleCounter = 3;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);
        
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        

        robot.motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //wobble
        //robot.wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive())
        {
            //drive

            //sets the power of the motors
            double LFpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed) );
            double LBpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed) );
            double RFpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed) );
            double RBpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed) );

            robot.motorRF.setPower(RFpower * speed);
            robot.motorRB.setPower(RBpower * speed);
            robot.motorLB.setPower(LBpower * speed);
            robot.motorLF.setPower(LFpower * speed);
            
            
            //turn speed toggle
            if (gamepad1.left_stick_button && leftStickToggle1) {
                leftStickToggle1 = false;

                if (!leftStickToggle2) {
                    turnSpeed = .5;
                    leftStickToggle2 = true;
                }

                else {
                    turnSpeed = 1;
                    leftStickToggle2 = false;
                }
              
            }

            else if (!gamepad1.left_stick_button && !leftStickToggle1) {
                leftStickToggle1 = true;
            }
            
            
            //drive speed toggle
            if (gamepad1.right_stick_button && rightStickToggle1) {
                rightStickToggle1 = false;

                if (!rightStickToggle2) {
                    driveSpeed = .5;
                    rightStickToggle2 = true;
                }

                else {
                    driveSpeed = 1;
                    rightStickToggle2 = false;
                }
              
            }

            else if (!gamepad1.right_stick_button && !rightStickToggle1) {
                rightStickToggle1 = true;
            }
            
            
            
            
            //flywheel

            robot.flywheel.setVelocityPIDFCoefficients( P , I , D , F);

            ((DcMotorEx) robot.flywheel).setVelocity(wheelSpeed);

            robot.servo.setPosition(servoPosition);

            //flywheel speed incress
            if (gamepad2.x && xToggle1) {
                xToggle1 = false;
                wheelSpeed -= 50;
              
            }

            else if (!gamepad2.x && !xToggle1) {
                xToggle1 = true;
            }
            
            
            //flywheel speed decress
            if (gamepad2.y && yToggle1) {
                yToggle1 = false;
                wheelSpeed += 50;
              
            }

            else if (!gamepad2.y && !yToggle1) {
                yToggle1 = true;
            }
            
            
            //flywheel on/off
            if (gamepad1.left_bumper && lBumperToggle1) {
                lBumperToggle1 = false;

                if (!lBumperToggle2) {
                    wheelSpeed = 1700;
                    lBumperToggle2 = true;
                }

                else {
                    wheelSpeed = 0;
                    lBumperToggle2 = false;
                }
              
            }

            else if (!gamepad1.left_bumper && !lBumperToggle1) {
                lBumperToggle1 = true;
            }


            //fire/reload
            if (gamepad1.a && aToggle1) {
                aToggle1 = false;

                if (!aToggle2) {
                    //fire
                    servoPosition = 0.115;

                    aToggle2 = true;
                }

                else {
                    servoPosition = 0.22;

                    aToggle2 = false;
                }
              
            }

            else if (!gamepad1.a && !aToggle1) {
                aToggle1 = true;
            }
            

            if(gamepad1.right_bumper)
            {
                ((DcMotorEx) robot.flywheel).setVelocity(1700);

                if (wheelSpeed < 1700)
                {
                    sleep(2000);
                }

                //shoot rings
                robot.servo.setPosition(.115);
                sleep(350);

                robot.servo.setPosition(0.22);
                sleep(550);

                robot.servo.setPosition(.115);
                sleep(350);

                robot.servo.setPosition(0.22);
                sleep(550);

                robot.servo.setPosition(.115);
                sleep(350);

                robot.servo.setPosition(0.22);

                //stop flywheel
                //((DcMotorEx) robot.flywheel).setVelocity(0);
            }
            
            
            
            if (gamepad1.b)
            {
                drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
                
                ((DcMotorEx) robot.flywheel).setVelocity(1600);
                
                Trajectory PowerShots = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-7, 15, Math.toRadians(-16)))
                .build();
                
                drive.followTrajectory(PowerShots);
                sleep(200);
                
                robot.servo.setPosition(.115);
                sleep(400);
                robot.servo.setPosition(0.22);
                
                drive.turn(Math.toRadians(-6));
                sleep(200);
                
                robot.servo.setPosition(.115);
                sleep(400);
                robot.servo.setPosition(0.22);

                ((DcMotorEx) robot.flywheel).setVelocity(1600);
                
                drive.turn(Math.toRadians(-6));
                sleep(200);
                
                robot.servo.setPosition(.115);
                sleep(400);
                robot.servo.setPosition(0.22);
                
                ((DcMotorEx) robot.flywheel).setVelocity(0);
            }
            
            //high goal: 9.4623 deg
            //left power shot: 12.7182 deg (16.25)
            // middle power shot: 17.5351 deg (22.75)
            //right power shot: 23.4622 deg (31.25)

            //wobble goal
            
            
            
            
            if (gamepad2.dpad_up && dPadUpToggle1) {
                dPadUpToggle1 = false;

                if (wobbleCounter < 3) {
                    wobbleCounter++;
                }
            }

            else if (!gamepad2.dpad_up && !dPadUpToggle1) {
                dPadUpToggle1 = true;
            }

            if (gamepad2.dpad_down && dPadDownToggle1) {
                dPadDownToggle1 = false;

                if (wobbleCounter > 1) {
                    wobbleCounter -= 1;
                }
            }

            else if (!gamepad2.dpad_down && !dPadDownToggle1) {
                dPadDownToggle1 = true;
            }


            if (-0.1 < (gamepad2.left_trigger - gamepad2.right_trigger) > 0.1) {
                robot.wobble.setPower(gamepad2.left_trigger/2 + -gamepad2.right_trigger/3);
            }
            
            else if (wobbleCounter == 2)
            {
                robot.wobble.setTargetPosition(upPosition);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            
            
            else if (wobbleCounter == 1)
            {
                robot.wobble.setTargetPosition(grabPosition);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            else if (wobbleCounter == 3)
            {
                robot.wobble.setTargetPosition(0);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad2.dpad_left)
            {
                robot.servo2.setPosition(openPosition);
            }

            if(gamepad2.dpad_right)
            {
                robot.servo2.setPosition(closedPosition);
            }

            //intake

            if (gamepad1.right_trigger > .5)
            {
                robot.intake1.setPower(-1);
                robot.intake2.setPower(1);
            }

            else if (gamepad1.left_trigger > .5)
            {
                robot.intake1.setPower(1);
                robot.intake2.setPower(-1);
            }

            else
            {
                robot.intake1.setPower(0);
                robot.intake2.setPower(0);
            }

            //telemetry

            //drive


                telemetry.addData("motor RF power", robot.motorRF.getPower());
                telemetry.addData("motor RF position", robot.motorRF.getCurrentPosition());

                telemetry.addData("motor RB power", robot.motorRB.getPower());
                telemetry.addData("motor RB position", robot.motorRB.getCurrentPosition());

                telemetry.addData("motor LF power", robot.motorLF.getPower());
                telemetry.addData("motor LF position", robot.motorLF.getCurrentPosition());

                telemetry.addData("motor LB power", robot.motorLB.getPower());
                telemetry.addData("motor LB position", robot.motorLB.getCurrentPosition());

                telemetry.addData("turnSpeed", turnSpeed);
                telemetry.addData("driveSpeed", driveSpeed);

            //shooter
                telemetry.addData("servo Pos", robot.servo.getPosition());
                telemetry.addData("servoPosition Var", servoPosition);
                telemetry.addData("wheelSpeed", wheelSpeed);
                telemetry.addData("current wheelSpeed", ((DcMotorEx) robot.flywheel).getVelocity());

            //wobble
                telemetry.addData("armPosition", robot.wobble.getCurrentPosition());
                telemetry.addData("servoPosition", robot.servo2.getPosition());

                telemetry.update();


        }
    }
}
