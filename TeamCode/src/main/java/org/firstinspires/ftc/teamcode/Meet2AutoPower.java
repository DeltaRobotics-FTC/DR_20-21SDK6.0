package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Meet2AutoPower")
public class Meet2AutoPower extends LinearOpMode {

    int shootingSpot = -3200;
    int park = -3750;
    int PowerStrafe = 600;
    double StrafeCorrection = .95;
    double flywheelSpeed = 1550;
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
        robot.motorLB.setTargetPosition(-6100);
        robot.motorLF.setTargetPosition(-6100);
        robot.motorRB.setTargetPosition(-6100);
        robot.motorRF.setTargetPosition(-6100);


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
        //robot.motorLB.setTargetPosition(shootingSpot);
        //robot.motorLF.setTargetPosition(shootingSpot);
        //robot.motorRB.setTargetPosition(shootingSpot);
        //robot.motorRF.setTargetPosition(shootingSpot);
//
//
        //robot.motorLB.setPower(0.4);
        //robot.motorLF.setPower(0.4);
        //robot.motorRB.setPower(0.4);
        //robot.motorRF.setPower(0.4);
//
        //robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
        //while (robot.motorLB.isBusy()) {}
//
        //sleep(1000);
//
        //robot.motorLB.setPower(0);
        //robot.motorLF.setPower(0);
        //robot.motorRB.setPower(0);
        //robot.motorRF.setPower(0);
      
        //start flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(flywheelSpeed);

        //strafe to power shot position
        robot.motorLB.setTargetPosition(-1600);
        robot.motorLF.setTargetPosition(-4800);
        robot.motorRB.setTargetPosition(-4800);
        robot.motorRF.setTargetPosition(-1600);

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

        while (((DcMotorEx) robot.flywheel).getVelocity() <= 1550) {
            telemetry.addData("LB target", "-2000");
            telemetry.addData("LF target", "-3800");
            telemetry.addData("LB pos", robot.motorLB.getCurrentPosition());
            telemetry.addData("LF pos", robot.motorLF.getCurrentPosition());
            telemetry.addData("flywheel speed", ((DcMotorEx) robot.flywheel).getVelocity());
            telemetry.update();
        }

        sleep(1000);

        //shoot rings
        robot.servo.setPosition(shoot);
        sleep(500);

        robot.servo.setPosition(0.25);

        //strafe to power shot position
        robot.motorLB.setTargetPosition(-800);
        robot.motorLF.setTargetPosition(-5100);
        robot.motorRB.setTargetPosition(-5300);
        robot.motorRF.setTargetPosition(-1000);

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
      
        //shoot rings
        robot.servo.setPosition(shoot);
        sleep(500);

        robot.servo.setPosition(0.25);

        //hihihihihihihihihihi

        //robot.servo.setPosition(shoot);
        //sleep(500);

        //robot.servo.setPosition(0.25);

        //stop flywheel
        ((DcMotorEx) robot.flywheel).setVelocity(0);

        //drive to the line
        //robot.motorLB.setTargetPosition(park);
        //robot.motorLF.setTargetPosition(park);
        //robot.motorRB.setTargetPosition(park);
        //robot.motorRF.setTargetPosition(park);


        //robot.motorLB.setPower(0.4);
        //robot.motorLF.setPower(0.4);
        //robot.motorRB.setPower(0.4);
        //robot.motorRF.setPower(0.4);

        //robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.motorLB.isBusy()) {}

        robot.motorLB.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorRB.setPower(0);
        robot.motorRF.setPower(0);
    }
}
