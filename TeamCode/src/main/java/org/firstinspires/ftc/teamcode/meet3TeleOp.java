package org.firstinspires.ftc.teamcode;

//drive
import com.acmerobotics.dashboard.FtcDashboard;
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


@TeleOp(name="meet3TeleOp" ,group = "")
public class meet3TeleOp extends LinearOpMode
{

    //drive
    double speed = 1.0;
    double zScale = 0.8;
    double turnSpeed = 1.0;
    double driveSpeed = 1.0;

    //flywheel
    double waitTime = 1000;
    double wheelSpeed = 0;
    double servoPosition = 0.21;

    public static double F = 13.32; // = 32767 / maxV      (do not edit from this number)
    public static double P = 20; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double I = 0;// = 0.1 * P           (fine ajustment of P)
    public static double D = 1; // = 0                     (raise to reduce ocolation)

    //wobble
    int upPosition = 300;
    int grabPosition = 825;
    double openPosition = 1;
    double closedPosition = 0;


    //intake
    double speed762590432128 = .75;

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        RobotHardware robot = new RobotHardware(hardwareMap);

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

            if (gamepad1.left_stick_button) {
                turnSpeed = 0.5;
            }

            else {
                turnSpeed = 1.0;
            }

            if (gamepad1.right_stick_button) {
                driveSpeed = 0.5;
            }

            else {
                driveSpeed = 1.0;
            }

            //flywheel



            //robot.flywheel.setVelocityPIDFCoefficients( P , I , D , F);

            ((DcMotorEx) robot.flywheel).setVelocity(wheelSpeed);

            robot.servo.setPosition(servoPosition);

            waitTime++;
            if (waitTime > 100)
            {

                if (gamepad1.x)
                {
                    wheelSpeed = wheelSpeed + 50;
                    waitTime = 0;
                }

                if (gamepad1.y)
                {
                    wheelSpeed = wheelSpeed - 50;
                    waitTime = 0;
                }
            }

            if(gamepad1.left_bumper)
            {
                wheelSpeed = 1700;
            }

            if(gamepad1.back)
            {
                wheelSpeed = 0;
            }

            if(gamepad1.back)
            {
                wheelSpeed = 0;
            }

            if (gamepad1.a)
            {
                servoPosition = 0.115;
            }
            if (gamepad1.b)
            {
                servoPosition = 0.21;
            }

            if(gamepad1.right_bumper)
            {
                ((DcMotorEx) robot.flywheel).setVelocity(1700);

                while (((DcMotorEx) robot.flywheel).getVelocity() != 1700) {}

                sleep(300);

                //shoot rings
                robot.servo.setPosition(.115);
                sleep(350);

                robot.servo.setPosition(0.21);
                sleep(550);

                robot.servo.setPosition(.115);
                sleep(350);

                robot.servo.setPosition(0.21);
                sleep(550);

                robot.servo.setPosition(.115);
                sleep(350);

                robot.servo.setPosition(0.21);

                //stop flywheel
                ((DcMotorEx) robot.flywheel).setVelocity(0);
            }

            //wobble goal
            // back up control : robot.wobble.setPower(gamepad1.left_trigger/2 + -gamepad1.right_trigger/3);
            
            if(gamepad1.dpad_up)
            {
                robot.wobble.setTargetPosition(upPosition);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            
            
            if(gamepad1.dpad_down)
            {
                robot.wobble.setTargetPosition(grabPosition);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.start)
            {
                robot.wobble.setTargetPosition(100);
                robot.wobble.setPower(0.7);
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.dpad_left)
            {
                robot.servo2.setPosition(openPosition);
            }

            if(gamepad1.dpad_right)
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
