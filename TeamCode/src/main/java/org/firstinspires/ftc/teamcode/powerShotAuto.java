package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "powerShotAuto")
public class powerShotAuto extends LinearOpMode {

    int shootingSpot = -2900;
    int park = -3607;
    double flywheelSpeed = 1600;
    double shoot = -.1;
    double looper = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Press Play to start op mode");
        //telemetry.addData("bla", robot.motorRB.getCurrentPosition();
        //telemetry.addData(robot.motorRF.getCurrentPosition());
        //telemetry.addData(robot.motorLF.getCurrentPosition());
        //telemetry.addData(robot.motorLB.getCurrentPosition());
        telemetry.update();
        waitForStart();

        //drive to shooting location
        robot.motorLB.setTargetPosition(shootingSpot);
        robot.motorLF.setTargetPosition(shootingSpot);
        robot.motorRB.setTargetPosition(shootingSpot);
        robot.motorRF.setTargetPosition(shootingSpot);


        robot.motorLB.setPower(0.4);
        robot.motorLF.setPower(0.4);
        robot.motorRB.setPower(0.4);
        robot.motorRF.setPower(0.4);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        while (robot.motorLB.isBusy()) {}

        sleep(1000);

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

        sleep(1000);

        //strafe to power shot position
        robot.motorLB.setTargetPosition(-2000);
        robot.motorLF.setTargetPosition(-3800);
        robot.motorRB.setTargetPosition(-3800);
        robot.motorRF.setTargetPosition(-2000);

        robot.motorLB.setPower(0.9);
        robot.motorLF.setPower(0.9);
        robot.motorRB.setPower(0.9);
        robot.motorRF.setPower(0.9);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

        while (looper >= 500)
        {
            telemetry.addData("LB target", "-2000");
            telemetry.addData("LF target", "-3800");
            telemetry.addData("LB pos", robot.motorLB.getCurrentPosition());
            telemetry.addData("LF pos", robot.motorLF.getCurrentPosition());
            telemetry.update();
            looper++;
        }

        //start flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelSpeed);

        while (((DcMotorEx) robot.flywheel).getVelocity() != 1600) {
            telemetry.addData("flywheel speed", ((DcMotorEx) robot.flywheel).getVelocity());
            telemetry.update();
        }

        sleep(3000);

        //shoot rings
        robot.servo.setPosition(shoot);
        sleep(2000);

        robot.servo.setPosition(0.25);
        sleep(1000);

        robot.motorLB.setTargetPosition(-1900);
        robot.motorLF.setTargetPosition(-3300);

        robot.motorLB.setPower(0.9);
        robot.motorLF.setPower(0.9);
        robot.motorRB.setPower(0.9);
        robot.motorRF.setPower(0.9);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

        looper = 0;
        while (looper >= 500)
        {
            telemetry.addData("LB target", "-1900");
            telemetry.addData("LF target", "-3300");
            telemetry.addData("LB pos", robot.motorLB.getCurrentPosition());
            telemetry.addData("LF pos", robot.motorLF.getCurrentPosition());
            telemetry.update();
            looper++;
        }

        robot.servo.setPosition(shoot);
        sleep(2000);

        robot.servo.setPosition(0.25);
        sleep(1000);

        robot.motorLB.setTargetPosition(-1800);
        robot.motorLF.setTargetPosition(-3200);

        robot.motorLB.setPower(0.9);
        robot.motorLF.setPower(0.9);
        robot.motorRB.setPower(0.9);
        robot.motorRF.setPower(0.9);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

        looper = 0;
        while (looper >= 500)
        {
            telemetry.addData("LB target", "-1800");
            telemetry.addData("LF target", "-3200");
            telemetry.addData("LB pos", robot.motorLB.getCurrentPosition());
            telemetry.addData("LF pos", robot.motorLF.getCurrentPosition());
            telemetry.update();
            looper++;
        }

        robot.servo.setPosition(shoot);
        sleep(2000);

        robot.servo.setPosition(0.25);

        //stop flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(0);

        //drive to the line
        robot.motorLB.setTargetPosition(park);
        robot.motorLF.setTargetPosition(park);
        robot.motorRB.setTargetPosition(park);
        robot.motorRF.setTargetPosition(park);


        robot.motorLB.setPower(0.4);
        robot.motorLF.setPower(0.4);
        robot.motorRB.setPower(0.4);
        robot.motorRF.setPower(0.4);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);
    }
}