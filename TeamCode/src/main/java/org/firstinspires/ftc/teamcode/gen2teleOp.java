 package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="gen2teleOp" ,group = "")
public class gen2teleOp extends LinearOpMode {


    public MecanumDriveTrain drive;
    //init variables

    public BNO055IMU imu;
    public void runOpMode() {
        MecanumDriveTrain drive = new MecanumDriveTrain(this);
            waitForStart();

            while (opModeIsActive()) {









            }


    }
}



