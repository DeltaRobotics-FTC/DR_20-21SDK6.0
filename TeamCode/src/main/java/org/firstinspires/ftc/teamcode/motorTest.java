package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motorTest", group = "")
public class motorTest extends LinearOpMode
{
    public void runOpMode() throws InterruptedException {
        int waitTime = 0;
        double motorSpeed1 = 0;
        double motorSpeed2 = 0;
        RobotHardware robot = new RobotHardware(hardwareMap);
        waitForStart();

        while(opModeIsActive())
        {

            robot.motorLF.setPower(motorSpeed2);
            robot.motorRF.setPower(motorSpeed1);
            waitTime++;
            if(waitTime > 1000)
            {
                if (gamepad1.dpad_up)
                {
                    motorSpeed1 += 0.005;
                }
                if (gamepad1.dpad_down)
                {
                    motorSpeed1 -= 0.005;
                }
                if(gamepad1.a)
                {
                    motorSpeed2 += 0.005;
                }
                if(gamepad1.b)
                {
                    motorSpeed2 -= 0.005;
                }
                telemetry.addData("motorspeed1", motorSpeed1);
                telemetry.addData("waitTime", waitTime);
                telemetry.update();
                waitTime = 0;
            }


        }
    }
}

