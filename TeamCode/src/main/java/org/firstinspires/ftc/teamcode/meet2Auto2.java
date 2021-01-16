package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "meet2Auto2")
public class meet2Auto2 extends LinearOpMode {

    int shootingSpot = -3200;
    int park = -3750;
    double flywheelSpeed = 1700;
    double shoot = -.1;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Press Play to start op mode");
        //telemetry.addData("bla", robot.motorRB.getCurrentPosition();
        //telemetry.addData(robot.motorRF.getCurrentPosition());
        //telemetry.addData(robot.motorLF.getCurrentPosition());
        //telemetry.addData(robot.motorLB.getCurrentPosition());
        telemetry.update();
        waitForStart();

        //drive to C goal location
        robot.motorLB.setTargetPosition(shootingSpot*2 + 000);
        robot.motorLF.setTargetPosition(shootingSpot*2 + 300);
        robot.motorRB.setTargetPosition(shootingSpot*2 + 300);
        robot.motorRF.setTargetPosition(shootingSpot*2 + 000);


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

        //start flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelSpeed);

        while (((DcMotorEx) robot.flywheel).getVelocity() != 1700) {}

        sleep(3000);

        //shoot rings
        robot.servo.setPosition(shoot);
        sleep(2000);

        robot.servo.setPosition(0.25);
        sleep(1000);

        robot.servo.setPosition(shoot);
        sleep(2000);

        robot.servo.setPosition(0.25);
        sleep(1000);

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