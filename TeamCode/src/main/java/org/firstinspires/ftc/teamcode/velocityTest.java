package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "velocityTest", group = "")
public class velocityTest extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        double waitTime = 1000;
        double speed = 1500;
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {
            ((DcMotorEx) robot.motorRF).setVelocity(speed);

            waitTime++;
            if(waitTime > 100)
            {

                if(gamepad1.dpad_up)
                {
                    speed = speed + 100;
                    waitTime = 0;
                }

                if(gamepad1.dpad_down)
                {
                    speed = speed - 100;
                    waitTime = 0;
                }



            }
            telemetry.addData("speed", speed);
            telemetry.addData("current speed", ((DcMotorEx) robot.motorRF).getVelocity());
            telemetry.update();
        }
    }
}
