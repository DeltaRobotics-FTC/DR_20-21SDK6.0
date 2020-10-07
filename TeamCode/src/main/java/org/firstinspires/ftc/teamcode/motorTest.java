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
        RobotHardware robot = new RobotHardware(hardwareMap);
        waitForStart();

        while(opModeIsActive())
        {

            robot.motorRF.setPower(motorSpeed1);
            if(gamepad1.dpad_up)
            {
            motorSpeed1 += 0.05;
            }
            if(gamepad1.dpad_down)
            {
            motorSpeed1 -= 0.05;
            }
            telemetry.addData("motorspeed1", motorSpeed1);
            telemetry.addData("waitTime", waitTime);
            telemetry.update();

            while(waitTime < 100)
            {
                waitTime++;
            }
            waitTime = 0;
        }
    }
}

