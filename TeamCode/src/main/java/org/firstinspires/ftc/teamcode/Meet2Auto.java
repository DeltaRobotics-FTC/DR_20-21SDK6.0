package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Meet2Auto")
public class Meet2Auto extends LinearOpMode {

    int shootingSpot = -3000;
    int park = 3500;
    double flywheelSpeed = 1700;
    double shoot = -.1;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        //drive to shooting location
        robot.motorLB.setTargetPosition(shootingSpot);
        robot.motorLF.setTargetPosition(shootingSpot);
        robot.motorRB.setTargetPosition(shootingSpot);
        robot.motorRF.setTargetPosition(shootingSpot);


        robot.motorLB.setPower(.4);
        robot.motorLF.setPower(.4);
        robot.motorRB.setPower(.4);
        robot.motorRF.setPower(.4);

        robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);

        //start flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelSpeed);

        while (((DcMotorEx) robot.flywheel).getVelocity() != 1700) {}

        //shoot rings
        //sleep(2000);
        robot.servo.setPosition(shoot);
        robot.servo.setPosition(0);
        //sleep(2000);
        robot.servo.setPosition(shoot);
        robot.servo.setPosition(0);
        //sleep(2000);
        robot.servo.setPosition(shoot);
        robot.servo.setPosition(0);

        //drive to C goal
        //robot.motorLB.setTargetPosition(shootingSpot*2);
        //robot.motorLF.setTargetPosition(shootingSpot*2);
        //robot.motorRB.setTargetPosition(shootingSpot*2);
        //robot.motorRF.setTargetPosition(shootingSpot*2);
//
//
        //robot.motorLB.setPower(.4);
        //robot.motorLF.setPower(.4);
        //robot.motorRB.setPower(.4);
        //robot.motorRF.setPower(.4);
//
        //robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
        //while (robot.motorLB.isBusy()) {}
//
        //robot.motorLB.setPower(0);
        //robot.motorLF.setPower(0);
        //robot.motorRB.setPower(0);
        //robot.motorRF.setPower(0);

        //stop flywheel
        //((DcMotorEx) robot.flywheel).setVelocity(0);

        //drive to the line
        //robot.motorLB.setTargetPosition(park);
        //robot.motorLF.setTargetPosition(park);
        //robot.motorRB.setTargetPosition(park);
        //robot.motorRF.setTargetPosition(park);
//
//
        //robot.motorLB.setPower(.4);
        //robot.motorLF.setPower(.4);
        //robot.motorRB.setPower(.4);
        //robot.motorRF.setPower(.4);
//
        //robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
        //while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);
    }
}
