package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="mecanumTeleOp" ,group = "")
public class mecanumTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
       double speed = 1.0;
       double zScale = 1.0;
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            robot.motorRF.setPower(speed*((-gamepad1.right_stick_y - gamepad1.right_stick_x) - (zScale * gamepad1.left_stick_x)));
            robot.motorRB.setPower(speed*(-(-gamepad1.right_stick_x + gamepad1.right_stick_y) - (zScale * gamepad1.left_stick_x)));
            robot.motorLB.setPower(speed*((gamepad1.right_stick_y + gamepad1.right_stick_x) - (zScale * gamepad1.left_stick_x)));
            robot.motorLF.setPower(speed*((-gamepad1.right_stick_x + gamepad1.right_stick_y)) - (zScale * gamepad1.left_stick_x));
        }
    }
}

