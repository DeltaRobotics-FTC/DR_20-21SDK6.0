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
    private Servo servo2;
    private DcMotor motorLB;


    @Override
    public void runOpMode()
    {
        motorLB = hardwareMap.dcMotor.get("flywheel");
        servo = hardwareMap.servo.get("servo");
        servo2 = hardwareMap.servo.get("servo2");
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double waitTime = 1000;
        double speed = 0;
        double servoPosition2 = 0;
        double servoPosition = 0.25;
        double change = 0.00556;
        boolean leftDPadState = false;
        boolean rightDPadState = false;




        waitForStart();

        while (opModeIsActive())
        {

            ((DcMotorEx) motorLB).setVelocity(speed);

            //motorLB.setPower(speed);

            //servo.setDirection(DcMotorSimple.Direction.FORWARD);
            servo.setPosition(servoPosition);
            servo2.setPosition(servoPosition2);
            waitTime++;
            if (waitTime > 100)
            {

                if (gamepad1.x)
                {
                    speed = speed + 50;
                    waitTime = 0;
                }

                if (gamepad1.y)
                {
                    speed = speed - 50;
                    waitTime = 0;
                }
                if (gamepad1.dpad_down){
                    servoPosition2 = servoPosition2 - change;
                    waitTime = 0;
                }
                if (gamepad1.dpad_up){
                    servoPosition2 = servoPosition2 + change;
                    waitTime = 0;
                }
            }

            if(gamepad1.left_bumper)
            {
                speed = 1700;
            }

            if (gamepad1.a)
            {
                servoPosition = 0.25;
            }
            if (gamepad1.b)
            {
                servoPosition = -0.25;
            }

            //if (gamepad1.dpad_up)
            //{
             //   servoPosition = 0;
            //}
            telemetry.addData("servo Pos", servo.getPosition());
            telemetry.addData("servo pos2", servo2.getPosition());
            telemetry.addData("servoPosition Var", servoPosition);
            telemetry.addData("servo2 position var", servoPosition2);
            telemetry.addData("speed", speed);
            telemetry.addData("current speed", ((DcMotorEx) motorLB).getVelocity());
            telemetry.update();
        }
    }
}
