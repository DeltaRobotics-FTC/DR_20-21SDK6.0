package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "velocityAndNormalServo", group = "")
public class velocityAndNormalServo extends LinearOpMode
{
    private Servo servo;
    private DcMotor motorLB;


    @Override
    public void runOpMode()
    {
        motorLB = hardwareMap.dcMotor.get("motorLB");
        servo = hardwareMap.servo.get("servo");
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double waitTime = 1000;
        double speed = 25000;
        double servoPosition = 0;
        boolean leftDPadState = false;
        boolean rightDPadState = false;

        boolean fastIncrement = true;


        waitForStart();

        while (opModeIsActive())
        {

            ((DcMotorEx) motorLB).setVelocity(speed);

            //servo.setDirection(DcMotorSimple.Direction.FORWARD);
            servo.setPosition(servoPosition);
            waitTime++;
            if (waitTime > 100)
            {

                if (gamepad1.x)
                {
                    speed = speed + 100;
                    waitTime = 0;
                }

                if (gamepad1.y)
                {
                    speed = speed - 100;
                    waitTime = 0;
                }
            }

            if (gamepad1.a)
            {
                servoPosition = 0.5;
            }
            if (gamepad1.b)
            {
                servoPosition = -0.5;
            }

            if (gamepad1.dpad_up)
            {
                servoPosition = 0;
            }
            telemetry.addData("servo Pos", servo.getPosition());
            telemetry.addData("servoPosition Var", servoPosition);
            telemetry.addData("Fast Increment", fastIncrement);
            telemetry.addData("speed", speed);
            telemetry.addData("current speed", ((DcMotorEx) motorLB).getVelocity());
            telemetry.update();
        }
    }
}
