package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="workingMecanum" ,group = "")
public class workingMecanum extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        double speed = 1.0;
        double zScale = 0.8;
        double turnSpeed = 1.0;
        double driveSpeed =1.0;
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {

            //sets the power of the motors
            double LFpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed));
            double LBpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed));
            double RFpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed));
            double RBpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed));

            robot.motorRF.setPower(RFpower * speed);
            robot.motorRB.setPower(RBpower * speed);
            robot.motorLB.setPower(LBpower * speed);
            robot.motorLF.setPower(LFpower * speed);
            
            if (gamepad1.left_stick_button) {
                turnSpeed = 1.0;
            }
            
            else {
                turnSpeed = .5;
            }
            
             if (gamepad1.right_stick_button) {
                driveSpeed = 1.0;
            }
            
            else {
                driveSpeed = .5;
            }
        }
    }
}
