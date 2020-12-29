package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


    @TeleOp(name="makeStuffWorking" ,group = "")
    public class makeStuffWorking extends LinearOpMode {


        @Override
        public void runOpMode() throws InterruptedException {
            double speed = 1.0;
            double zScale = 1.0;
            double motorSpeed1 = 0;
            double waitTime = 0;
            RobotHardware robot = new RobotHardware(hardwareMap);

            waitForStart();

            while (opModeIsActive()) {

                //sets the power of the motors
                robot.motorRF.setPower(-speed*((-gamepad1.left_stick_y - gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
                robot.motorRB.setPower(-speed*(-(-gamepad1.left_stick_x + gamepad1.left_stick_y) - (zScale * gamepad1.right_stick_x)));
                robot.motorLB.setPower(-speed*((gamepad1.left_stick_y + gamepad1.left_stick_x) - (zScale * gamepad1.right_stick_x)));
                robot.motorLF.setPower(-speed*((-gamepad1.left_stick_x + gamepad1.left_stick_y)) - (zScale * gamepad1.right_stick_x));

//if statement reduces/increases motor power accordingly if a motor has more than a power of 1 or less than a power of -1
//that way all the motors remain proportional but at the highest speed possible forward or reverse
//if you move slowly there is nothing to reduce and it will still go slowly
/*
                robot.intake1.setPower(motorSpeed1);
                waitTime++;
                if (waitTime > 1000) {
                    if (gamepad1.dpad_up) {
                        motorSpeed1 += 0.005;
                    }
                    if (gamepad1.dpad_down) {
                        motorSpeed1 -= 0.005;
                    }

*/

//robot power is your speed multiplier




  //              }
            }
        }
    }



