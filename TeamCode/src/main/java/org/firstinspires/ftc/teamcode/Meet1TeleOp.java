package org.firstinspires.ftc.teamcode;

//drive
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//flywheel
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//wobble

@TeleOp(name="Meet1TeleOp" ,group = "")
public class Meet1TeleOp extends LinearOpMode
{

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

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            //drive

            //sets the power of the motors
            double LFpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed));
            double LBpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed));
            double RFpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed));
            double RBpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed));

            robot.motorRF.setPower(RFpower * speed);
            robot.motorRB.setPower(RBpower * speed);
            robot.motorLB.setPower(LBpower * speed);
            robot.motorLF.setPower(LFpower * speed);

            //fix gamepad buttons
            if (gamepad1.left_stick_button == true) {
                turnSpeed = 1.0;
            }

            else {
                turnSpeed = .5;
            }

            if (gamepad1.right_stick_button == true) {
                driveSpeed = 1.0;
            }

            else {
                driveSpeed = .5;
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

            telemetry.addData("servo Pos", robot.servo.getPosition());
            telemetry.addData("servoPosition Var", servoPosition);
            telemetry.addData("speed", speed);
            telemetry.addData("current speed", ((DcMotorEx) robot.flywheel).getVelocity());
            telemetry.update();

            //wobble
            if(gamepad1.dpad_up)
            {
                //motor up
            }

            if(gamepad1.dpad_down)
            {
                //motor down
            }

            if(gamepad1.dpad_left)
            {
                //servo open
            }

            if(gamepad1.dpad_right)
            {
                //servo close
            }
        }
    }
}
