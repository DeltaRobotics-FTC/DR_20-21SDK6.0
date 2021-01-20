package org.firstinspires.ftc.teamcode.VelocityTuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MaxVelocity")
public class MaxVelocity extends LinearOpMode
{
    DcMotorEx flywheel;
    double currentVelocity;
    double maxVelocity = 0.000;
 
    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        waitForStart();

        flywheel.setPower(1);
 
        while (opModeIsActive()) {
            currentVelocity = flywheel.getVelocity();
            
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
