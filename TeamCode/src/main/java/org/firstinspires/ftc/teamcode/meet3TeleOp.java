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

    public static double F = 13.32;
    public static double P = 20;
    public static double I = 0;
    public static double D = 1;

    //drive
    double speed = 1.0;
    double zScale = 0.8;
    double turnSpeed = 1.0;
    double driveSpeed = 1.0;

    //flywheel
    double waitTime = 1000;
    double wheelSpeed = 0;
    double servoPosition = 0.25;

    //wobble
    int upPosition = 850;
    int grabPosition = 350;
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

        //wobble
        robot.wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

            if (gamepad1.left_stick_button == true) {
                turnSpeed = 0.5;
            }

            else {
                turnSpeed = 1.0;
            }

            if (gamepad1.right_stick_button == true) {
                driveSpeed = 0.5;
            }

            else {
                driveSpeed = 1.0;
            }

            //flywheel

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

            if (gamepad1.a)
            {
                servoPosition = 0.25;
            }
            if (gamepad1.b)
            {
                servoPosition = -0.1;
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
                robot.wobble.setTargetPosition(0);
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
            //robot.intake1.setPower(-gamepad1.leftTriger + gamepad1.rightTriger);
            //robot.intake2.setPower(-gamepad1.rightTriger + gamepad1.leftTriger);

            //telemetry

            //drive
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
