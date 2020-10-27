package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="mecanumTeleOp" ,group = "")
public class mecanumTeleOp extends LinearOpMode {

    //encoder counts per in of movement (counts per rotation / pi*r^2
    final double COUNTS_PER_INCH = 1312.54037886341;

    //OdometryGlobalCoordinatePosition is the thread
//globalPositionThread is a variable that will hold the thread with specific info like the names of the encoders
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
       double speed = 1.0;
       double zScale = 1.0;
        RobotHardware robot = new RobotHardware(hardwareMap);

        //stopEncoder();

        //fills the thread variable with the thread with encoder names how many ticks per in and the delay
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        waitForStart();

        while (opModeIsActive())
        {

            //sets the power of the motors
            double LFpower = ( -gamepad1.right_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x) * speed;
            double LBpower = ( -gamepad1.right_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x) * speed;
            double RFpower = ( -gamepad1.right_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x) * speed;
            double RBpower = ( -gamepad1.right_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x) * speed;

//if statement reduces/increases motor power accordingly if a motor has more than a power of 1 or less than a power of -1
//that way all the motors remain proportional but at the highest speed possible forward or reverse
//if you move slowly there is nothing to reduce and it will still go slowly

            double motorPowerRatio = 1;

            if (LFpower >= 1 && LFpower >= LBpower && LFpower >= RFpower && LFpower >= RBpower)
            {
                motorPowerRatio = 1 / LFpower;
            } else if (LBpower >= 1 && LBpower >= LFpower && LBpower >= RFpower && LBpower >= RBpower)
            {
                motorPowerRatio = 1 / LBpower;
            } else if (RFpower >= 1 && RFpower >= LFpower && RFpower >= LBpower && RFpower >= RBpower)
            {
                motorPowerRatio = 1 / RFpower;
            } else if (RBpower >= 1 && RBpower >= LFpower && RBpower >= RFpower && RBpower >= LBpower)
            {
                motorPowerRatio = 1 / RBpower;
            } else if (LFpower <= -1 && LFpower <= LBpower && LFpower <= RFpower && LFpower <= RBpower)
            {
                motorPowerRatio = -1 / LFpower;
            } else if (LBpower <= -1 && LBpower <= LFpower && LBpower <= RFpower && LBpower <= RBpower)
            {
                motorPowerRatio = -1 / LBpower;
            } else if (RFpower <= -1 && RFpower <= LFpower && RFpower <= LBpower && RFpower <= RBpower)
            {
                motorPowerRatio = -1 / RFpower;
            } else if (RBpower <= -1 && RBpower <= LFpower && RBpower <= RFpower && RBpower <= LBpower)
            {
                motorPowerRatio = -1 / RBpower;
            }



//robot power is your speed multiplier

            robot.motorRF.setPower(RFpower * motorPowerRatio);
            robot.motorRB.setPower(RBpower * motorPowerRatio);
            robot.motorLB.setPower(LBpower * motorPowerRatio);
            robot.motorLF.setPower(LFpower * motorPowerRatio);

            telemetry.addData("orientation", globalPositionUpdate.returnOrientation());
            telemetry.addData("X cord", globalPositionUpdate.returnXCoordinate());
            telemetry.addData("Y cord", globalPositionUpdate.returnYCoordinate());
            telemetry.update();
        }



    }

    public void stopEncoder() {

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}

