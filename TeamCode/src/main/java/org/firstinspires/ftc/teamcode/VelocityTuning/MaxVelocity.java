package org.firstinspires.ftc.teamcode.MaxVelocity;
 
@TeleOp
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    double currentVelocity;
    double maxVelocity = 0.000;
 
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "flywheel");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        waitForStart();
        
        motorRF.setPower(1);
 
        while (opModeIsActive()) {
            currentVelocity = motor.getVelocity();
            
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
