package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "velocityAndServo", group = "")
public class velocityAndServo extends LinearOpMode
{
    private CRServo servo;
    private DcMotor motorRF;



    @Override
    public void runOpMode()
    {
        motorRF = hardwareMap.dcMotor.get("motorRF");
        servo = hardwareMap.crservo.get("servo");
        double waitTime = 1000;
        double speed = 0;

        boolean leftDPadState = false;
        boolean rightDPadState = false;

        boolean fastIncrement = true;




        waitForStart();

        while(opModeIsActive())
        {
            double servoPower = 0.5;
            ((DcMotorEx) motorRF).setVelocity(speed);

            //servo.setDirection(DcMotorSimple.Direction.FORWARD);
            servo.setPower(.5);
            waitTime++;
            if(waitTime > 100)
            {

                if(gamepad1.x)
                {
                    speed = speed + 100;
                    waitTime = 0;
                }

                if(gamepad1.y)
                {
                    speed = speed - 100;
                    waitTime = 0;
                }
                if(gamepad1.a)
                {
                    servoPower = 0.5;
                    waitTime = 0;
                }
                if(gamepad1.dpad_right)
                {
                    servoPower = .9;
                    waitTime = 0;
                }
                if(gamepad1.dpad_left)
                {
                    servoPower = .4;
                    waitTime = 0;

                }


            }







            telemetry.addData("servo Pos", servo.getPower());
            telemetry.addData("servoPosition Var", servoPower);
            telemetry.addData("Fast Increment", fastIncrement);
            telemetry.addData("speed", speed);
            telemetry.addData("current speed", ((DcMotorEx) motorRF).getVelocity());
            telemetry.update();
        }
    }
}

