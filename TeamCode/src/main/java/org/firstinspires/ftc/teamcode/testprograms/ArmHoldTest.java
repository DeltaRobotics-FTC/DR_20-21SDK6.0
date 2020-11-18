package org.firstinspires.ftc.teamcode.testprograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class ArmHoldTest extends LinearOpMode
{
    //Robot robot = new Robot(hardwareMap);

    DcMotor arm;
    //DcMotor arm2;

    int targetPosition;

    boolean dDpadLeftState = false;
    boolean dPadRightState = false;
    boolean dPadUpState = false;
    boolean dPadDownState = false;

    double maxMotorPower = 0.5;
    int nextTargetPos  = 0;

    @Override
    public void runOpMode()
    {
        arm = hardwareMap.dcMotor.get("arm");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetPosition = arm.getCurrentPosition();

        arm.setTargetPosition(targetPosition);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        /*arm2 = hardwareMap.dcMotor.get("arm2");

        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm2.setTargetPosition(targetPosition);

        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);/*/



        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.dpad_up && !dPadUpState)
            {
                dPadUpState = true;

                if(maxMotorPower < 1.0)
                {
                    maxMotorPower += 0.05;
                }
            }
            else if (!gamepad1.dpad_up)
            {
                dPadUpState = false;
            }

            if(gamepad1.dpad_down && !dPadDownState)
            {
                dPadDownState = true;

                if(maxMotorPower > -1.0)
                {
                    maxMotorPower -= 0.05;
                }
            }
            else if(!gamepad1.dpad_down)
            {
                dPadDownState = false;
            }

            if(gamepad1.dpad_left && !dDpadLeftState)
            {
                dDpadLeftState = true;

                nextTargetPos += 5;
            }
            else if (!gamepad1.dpad_left)
            {
                dDpadLeftState = false;
            }

            if(gamepad1.dpad_right && !dPadRightState)
            {
                dPadRightState = true;

                nextTargetPos -= 5;
            }
            else if(!gamepad1.dpad_right)
            {
                dPadRightState = false;
            }

            if(gamepad1.a)
            {
                targetPosition = nextTargetPos;
            }

            /*if(gamepad1.a)
            {
                robot.blockMover.grabber.raiseArm();
            }

            if(gamepad1.b)
            {
                robot.blockMover.grabber.lowerArm();
            }*/

            arm.setTargetPosition(targetPosition);
            arm.setPower(maxMotorPower);

            //arm2.setTargetPosition(targetPosition);
            //arm2.setPower(maxMotorPower);


            telemetry.addData("Current Target Arm", arm.getTargetPosition());
            //telemetry.addData("Current Target Arm2", arm2.getTargetPosition());
            telemetry.addData("Next Target Position", targetPosition);
            telemetry.addData("Next Target", nextTargetPos);
            telemetry.addData("Current Position Arm", arm.getCurrentPosition());
            //telemetry.addData("Current Position Arm2", arm2.getCurrentPosition());
            telemetry.addData("Motor Power Arm", arm.getPower());
            //telemetry.addData("Motor Power Arm2", arm2.getPower());
            telemetry.addData("Max Motor Power", maxMotorPower);
            //telemetry.addData("FirstMeetBlockMover pos", robot.blockMover.grabber.getServo().getPosition());
            telemetry.update();
        }
    }
}
