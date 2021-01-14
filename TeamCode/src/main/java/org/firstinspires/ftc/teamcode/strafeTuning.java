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

  int shootingSpot = 4000;
  double strafeSpeedMultiplier = 1.1;
  double strafeDistanceMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        
        //if multiplying shooting spot you must round and keep the number as an interger
        robot.motorLB.setTargetPosition(-shootingSpot);
        robot.motorLF.setTargetPosition(shootingSpot);
        robot.motorRB.setTargetPosition(shootingSpot);
        robot.motorRF.setTargetPosition(-shootingSpot);


        robot.motorLB.setPower(0.4 * (1/strafeSpeedMultiplier));
        robot.motorLF.setPower(0.4 * strafeSpeedMultiplier);
        robot.motorRB.setPower(0.4 * (1/strafeSpeedMultiplier));
        robot.motorRF.setPower(0.4 * strafeSpeedMultiplier);

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
    }
}
