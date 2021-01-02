package org.firstinspires.ftc.teamcode.testprograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTest", group = "")
@Disabled
public class servoTest extends LinearOpMode
{
    private Servo servo;


    @Override
    public void runOpMode()
    {
        servo = hardwareMap.servo.get("servo");
        double waitTime = 1000;
        double servoPosition = 0;

        boolean fastIncrement = true;


        waitForStart();

        while (opModeIsActive())
        {
            servo.setPosition(servoPosition);
            waitTime++;

            if (gamepad1.a)
            {
                servoPosition = servoPosition + 0.1;
            }
            if (gamepad1.b)
            {
                servoPosition = servoPosition - 0.1;
            }

            if (gamepad1.dpad_up)
            {
                servoPosition = 0;
            }
            telemetry.addData("servo Pos", servo.getPosition());
            telemetry.addData("servoPosition Var", servoPosition);
            telemetry.addData("Fast Increment", fastIncrement);
            telemetry.update();
        }
    }
}
