package org.firstinspires.ftc.teamcode.oldOdometry;

//import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "customOdometryTestV2")
@Disabled
public class customOdometryTestV2 extends LinearOpMode
{

    //encoder counts per in of movement (counts per rotation / pi*r^2
    final double COUNTS_PER_INCH = 1312.54037886341;

    //OdometryGlobalCoordinatePosition is the thread
//globalPositionThread is a variable that will hold the thread with specific info like the names of the encoders
    //OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException
    {

//call hardware map
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

//fills the thread variable with the thread with encoder names how many ticks per in and the delay
        //globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        //Thread positionThread = new Thread(globalPositionUpdate);
        //positionThread.start();

        goToPosition(5,5,0);

        while (opModeIsActive())
        {

//Display Global (x, y, theta) coordinates
            //telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            //telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            //telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

//Display encoder values
            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("motorRF power", robot.motorRF.getPower());
            telemetry.addData("motorRB power", robot.motorRB.getPower());
            telemetry.addData("motorLF power", robot.motorLF.getPower());
            telemetry.addData("motorLB power", robot.motorLB.getPower());

            //telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

//Stop the thread
        //globalPositionUpdate.stop();

    }

    public void goToPosition (double XTargetInches, double YTargetInches, double power){

        RobotHardware robot = new RobotHardware(hardwareMap);

        double XTargetTicks = XTargetInches * COUNTS_PER_INCH;
        double YTargetTicks = YTargetInches * COUNTS_PER_INCH;

        //double XDistanceTicks = XTargetTicks - globalPositionUpdate.returnXCoordinate();
        //double YDistanceTicks = YTargetTicks - globalPositionUpdate.returnYCoordinate();

        //double robotMovementAngle = Math.toDegrees(Math.atan2(XDistanceTicks, YDistanceTicks));
    }
}
