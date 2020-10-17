package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "customOdometryTest")
public class customOdometryTest extends LinearOpMode
{

    //encoder counts per in of movement (counts per rotation / pi*r^2
    final double COUNTS_PER_INCH = 2607.59459;

    //OdometryGlobalCoordinatePosition is the thread
//globalPositionThread is a variable that will hold the thread with specific info like the names of the encoders
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException
    {

//call hardware map
        RobotHardware robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

//fills the thread variable with the thread with encoder names how many ticks per in and the delay
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        goToPosition(24, 0, .25, 0, 1.5);


        while(opModeIsActive())
        {

//Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

//Display encoder values
            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

//Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double robotOrientation, double allowableDistanceError)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        targetXPosition = targetXPosition * COUNTS_PER_INCH;
        targetYPosition = targetYPosition * COUNTS_PER_INCH;
        allowableDistanceError = allowableDistanceError * COUNTS_PER_INCH;

        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while(opModeIsActive() &&  distance > allowableDistanceError)
        {


            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            double pivotCorectionAngle = robotOrientation - globalPositionUpdate.returnOrientation();
            double pivotCorectionPower = pivotCorectionAngle / 180;


            //sets the power of the motors
            double LFpower = robotMovementXComponent + robotMovementYComponent + pivotCorectionPower;
            double LBpower = robotMovementXComponent - robotMovementYComponent + pivotCorectionPower;
            double RFpower = robotMovementXComponent - robotMovementYComponent - pivotCorectionPower;
            double RBpower = robotMovementXComponent + robotMovementYComponent - pivotCorectionPower;

//if statement reduces/increases motor power accordingly if a motor has more than a power of 1 or less than a power of -1
//that way all the motors remain proportional but at the highest speed possible forward or reverse
//if you move slowly there is nothing to reduce and it will still go slowly

            double motorPowerRatio = 1;

            if(LFpower >= 1 && LFpower >= LBpower && LFpower >= RFpower && LFpower >= RBpower)
            {
                motorPowerRatio = 1 / LFpower;
            }

            else if(LBpower >= 1 && LBpower >= LFpower && LBpower >= RFpower && LBpower >= RBpower)
            {
                motorPowerRatio = 1 / LBpower;
            }

            else if(RFpower >= 1 && RFpower >= LFpower && RFpower >= LBpower && RFpower >= RBpower)
            {
                motorPowerRatio = 1 / RFpower;
            }

            else if(RBpower >= 1 && RBpower >= LFpower && RBpower >= RFpower && RBpower >= LBpower)
            {
                motorPowerRatio = 1 / RBpower;
            }

            else if(LFpower <= -1 && LFpower <= LBpower && LFpower <= RFpower && LFpower <= RBpower)
            {
                motorPowerRatio = -1 / LFpower;
            }

            else if(LBpower <= -1 && LBpower <= LFpower && LBpower <= RFpower && LBpower <= RBpower)
            {
                motorPowerRatio = -1 / LBpower;
            }

            else if(RFpower <= -1 && RFpower <= LFpower && RFpower <= LBpower && RFpower <= RBpower)
            {
                motorPowerRatio = -1 / RFpower;
            }

            else if(RBpower <= -1 && RBpower <= LFpower && RBpower <= RFpower && RBpower <= LBpower)
            {
                motorPowerRatio = -1 / RBpower;
            }


//robot power is your speed multiplier

            robot.motorRF.setPower(robotPower * RFpower * motorPowerRatio);
            robot.motorRB.setPower(robotPower * RBpower * motorPowerRatio);
            robot.motorLB.setPower(robotPower * LBpower * motorPowerRatio);
            robot.motorLF.setPower(robotPower * LFpower * motorPowerRatio);


        }
    }
    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

}
