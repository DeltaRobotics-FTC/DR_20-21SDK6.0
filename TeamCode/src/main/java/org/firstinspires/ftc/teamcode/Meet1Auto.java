package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Meet1Auto")
public class Meet1Auto extends LinearOpMode {

    public MecanumDriveTrain drive;

    public BNO055IMU imu;

    int shootingSpot = 2029;
    int park = 3607 - shootingSpot;
    double flywheelSpeed = 1700;
    double shoot = -.1;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDriveTrain drive = new MecanumDriveTrain(this);
        
        RobotHardware robot = new RobotHardware(hardwareMap);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

//drive to shooting location
        drive.encoderDrive(shootingSpot,driveStyle.BACKWARD,.4);


        //start flywheel
        ((DcMotorEx) flywheel).setVelocity(flywheelSpeed);

        while (((DcMotorEx) flywheel).getVelocity() != 1700) {
            sleep(1);
        }
        //shoot rings
            servo.setPosition(shoot);
            servo.setPosition(0);
            sleep(100);
            servo.setPosition(shoot);
        servo.setPosition(0);
        sleep(100);
        servo.setPosition(shoot);
        servo.setPosition(0);

        //stop flywheel
        ((DcMotorEx) flywheel).setVelocity(0);

        //drive to the line
        drive.encoderDrive(park,driveStyle.BACKWARD,.4);

        //drive.OrientationDrive(360,.7, imu);
    }
}
