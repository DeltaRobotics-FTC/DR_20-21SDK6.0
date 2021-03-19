package org.firstinspires.ftc.teamcode.VelocityTuning;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class velocityPIDFTuner extends LinearOpMode
{
    DcMotorEx motor;

    //do not put equasions in for PIDF variables, enter the answers to the math.
    //the numbers from the equasions are starting points
    public static double F = 13.32; // = 32767 / maxV      (do not edit from this number)
    public static double P = 20; // = 0.1 * F           (raise till real's apex touches Var apex)
    public static double I = 0;// = 0.1 * P           (fine ajustment of P)
    public static double D = 1; // = 0                     (raise to reduce ocolation)

    double wheelSpeed = 1700;
    double speed = 0;
    double speed2 = 0;
    double speed3 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime timer = new ElapsedTime();

        //this.velocityEstimates = new MovingStatistics(100);

        waitForStart();

        while (opModeIsActive()) {

            motor.setVelocityPIDFCoefficients( P , I , D , F);

            timer.reset();

            ((DcMotorEx) motor).setVelocity(wheelSpeed);

            while (timer.seconds() < 5) {
                speed3 = speed2;
                speed2 = speed;
                speed = motor.getVelocity();
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", ((speed3 + speed2 +speed)/3));
                telemetry.update();
            }

            ((DcMotorEx) motor).setVelocity(0);



            while (timer.seconds() < 20) {
                telemetry.addData("wheelSpeedVar", wheelSpeed);
                telemetry.addData("wheelSpeedReal", ((DcMotorEx) motor).getVelocity());
                telemetry.update();
            }
        }
    }
}
