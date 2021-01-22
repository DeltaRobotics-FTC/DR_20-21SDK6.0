package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class servoShootingSpeed extends LinearOpMode
{
    DcMotorEx motor;
    Servo servo;

    //do not put equasions in for PIDF variables, enter the answers to the math.
    //the numbers from the equasions are starting points
    public static double F = 13.32; // = 32767 / maxV      (do not edit from this number)
    public static double P = 20; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double I = 0;// = 0.1 * P           (fine ajustment of P)
    public static double D = 1; // = 0                     (raise to reduce ocolation)

    public static double wheelSpeed = 1700;
    
    public static boolean test = true;
    public static double shoot = -1;
    public static double back = .2;
    
    double nanoTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo = hardwareMap.servo.get("servo");
        servo.setPosition(shoot);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        motor.setVelocityPIDFCoefficients( P , I , D , F);
        
        while (opModeIsActive()) {
        
            ((DcMotorEx) motor).setVelocity(wheelSpeed);
            while (test) {
                servo.setPosition(shoot);
            }
            
            while (test != true) {
                servo.setPosition(back);
            }

            timer.reset();

            servo.setPosition(shoot);
            
            while (timer.milliseconds() < 250) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", (motor.getVelocity()));
                telemetry.update();
            }
            
            servo.setPosition(back);
            
            while (timer.milliseconds() < 500) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", (motor.getVelocity()));
                telemetry.update();
            }
            
            servo.setPosition(shoot);
            
            while (timer.milliseconds() < 750) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", (motor.getVelocity()));
                telemetry.update();
            }
            
            servo.setPosition(back);
            
            while (timer.milliseconds() < 1000) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", (motor.getVelocity()));
                telemetry.update();
            }
            
            servo.setPosition(shoot);
            
            while (timer.milliseconds() < 1250) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", (motor.getVelocity()));
                telemetry.update();
            }
            
            servo.setPosition(back);
            
            nanoTime = timer.nanoseconds();
            
            while (test) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", (motor.getVelocity()));
                telemetry.addData("time", nanoTime);
                telemetry.update();
            }
        }
    }
}
