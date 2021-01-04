package org.firstinspires.ftc.teamcode;

//drive
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//flywheel
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//wobble

@TeleOp(name="Meet1TeleOp" ,group = "")
public class Meet1TeleOp extends LinearOpMode
{

    //drive
    double speed = 1.0;
    double zScale = 0.8;
    double turnSpeed = 1.0;
    double driveSpeed = 1.0;

    //flywheel
    double waitTime = 1000;
    double wheelSpeed = 0;
    double servoPosition = 0.25;

    //wobble
    int upPosition = 454;
    int grabPosition = 227;
    double openPosition = 1;
    double closedPosition = -1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware robot = new RobotHardware(hardwareMap);
        
        wobble.setTargetPosition(0)
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive())
        {
            //drive

            //sets the power of the motors
            double LFpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed));
            double LBpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) + (gamepad1.left_stick_x * zScale * turnSpeed));
            double RFpower = ( ((-gamepad1.right_stick_y - gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed));
            double RBpower = ( ((-gamepad1.right_stick_y + gamepad1.right_stick_x) * driveSpeed) - (gamepad1.left_stick_x * zScale * turnSpeed));

            robot.motorRF.setPower(RFpower * speed);
            robot.motorRB.setPower(RBpower * speed);
            robot.motorLB.setPower(LBpower * speed);
            robot.motorLF.setPower(LFpower * speed);

            if (gamepad1.left_stick_button == true) {
                turnSpeed = 1.0;
            }

            else {
                turnSpeed = .5;
            }

            if (gamepad1.right_stick_button == true) {
                driveSpeed = 1.0;
            }

            else {
                driveSpeed = .5;
            }

            //flywheel

            ((DcMotorEx) robot.flywheel).setVelocity(wheelSpeed);

            robot.servo.setPosition(servoPosition);

            waitTime++;
            if (waitTime > 100)
            {

                if (gamepad1.x)
                {
                    wheelSpeed = wheelSpeed + 50;
                    waitTime = 0;
                }

                if (gamepad1.y)
                {
                    wheelSpeed = wheelSpeed - 50;
                    waitTime = 0;
                }
            }

            if(gamepad1.left_bumper)
            {
                wheelSpeed = 1700;
            }

            if (gamepad1.a)
            {
                servoPosition = 0.25;
            }
            if (gamepad1.b)
            {
                servoPosition = -0.1;
            }

            //wobble
            if(gamepad1.dpad_up)
            {
                robot.wobble.setTargetPosition(upPosition)
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSIOION)
                
                //upPosition
            }

            if(gamepad1.dpad_down)
            {
                robot.wobble.setTargetPosition(grabPosition)
                robot.wobble.setMode(DcMotor.RunMode.RUN_TO_POSIOION)
                
                //grabPosition
            }

            if(gamepad1.dpad_left)
            {
                robot.servo2.setPosition(openPosition);
            }

            if(gamepad1.dpad_right)
            {
                robot.servo2.setPosition(closedPosition)
            }
            
            //drive wobble arm (if encoders are off)
            //eventualy replace with intake
            //robot.wobble.setPower(-gamepad1.leftTriger + gamepad1.rightTriger);
            
            //telemetry
            
            //drive
            telemetry.addData("turnSpeed", turnSpeed);
            telemetry.addData("driveSpeed", driveSpeed);
            
            //shooter
            telemetry.addData("servo Pos", robot.servo.getPosition());
            telemetry.addData("servoPosition Var", servoPosition);
            telemetry.addData("wheelSpeed", wheelSpeed);
            telemetry.addData("current wheelSpeed", ((DcMotorEx) robot.flywheel).getVelocity());
            
            //wobble
            telemetry.addData("armPosition", robot.wobble.getCurrentPosition());
            telemetry.addData("servoPosition", robot.servo2.getPosition());
            
            telemetry.update();
        }
    }
}
