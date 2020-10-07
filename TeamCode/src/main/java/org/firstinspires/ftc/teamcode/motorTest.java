package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "motorTest", group = "")
public class motorTest extends LinearOpMode
{
    public void runOpMode()
    {

        double motorSpeed = 0;
        boolean upstate = true;
        boolean downstate = false;
        boolean Astate = true;
        boolean state = false;
        RobotHardware robot = new RobotHardware(hardwareMap);
        waitForStart();

        while(opModeIsActive())
        {

            robot.motorRF.setPower(motorSpeed);
            if(gamepad1.dpad_up)
            {
            upstate = false;
            }
            else
            {
            upstate = true;
            }
            if(!upstate && !downstate)
            {
            motorSpeed += 0.125;
            downstate = true;
            }
            else
            {
            downstate = false;
            }
            telemetry.addData("motorspeed", motorSpeed);
            telemetry.addData("upstate", upstate);
            telemetry.addData("downstate", downstate);
            telemetry.update();

        }
    }
}

