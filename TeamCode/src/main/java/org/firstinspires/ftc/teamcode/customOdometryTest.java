package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.OdometryGlobalCoordinatePosition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "customOdometryTest")
public class customOdometryTest extends LinearOpMode
{

    //encoder counts per in of movement (counts per rotation / pi*r^2
    final double COUNTS_PER_INCH = 1312.54037886341;

    //OdometryGlobalCoordinatePosition is the thread
//globalPositionThread is a variable that will hold the thread with specific info like the names of the encoders
    OdometryGlobalCoordinatePosition globalPositionUpdate;

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
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.verticalLeft, robot.verticalRight, robot.horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        goToPosition(0, 0, 1, 90, .5, 1);

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

            telemetry.addData("motorRF power", robot.motorRF.getPower());
            telemetry.addData("motorRB power", robot.motorRB.getPower());
            telemetry.addData("motorLF power", robot.motorLF.getPower());
            telemetry.addData("motorLB power", robot.motorLB.getPower());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

//Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double robotOrientation, double allowableDistanceError, double allowableOrientationError)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        boolean runLoop = true;

        targetXPosition = targetXPosition * COUNTS_PER_INCH;
        targetYPosition = targetYPosition * COUNTS_PER_INCH;
        allowableDistanceError = allowableDistanceError * COUNTS_PER_INCH;

        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        while(opModeIsActive() &&  runLoop)
        {
            double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

            double pivotCorectionAngle = robotOrientation - globalPositionUpdate.returnOrientation();

            if (distance > allowableDistanceError || pivotCorectionAngle > allowableOrientationError)
            {
                runLoop = true;
            }

            else
            {
                runLoop = false;
            }


            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robotMovementXComponent = calculateX(robotMovementAngle, robotPower);
            double robotMovementYComponent = calculateY(robotMovementAngle, robotPower);

            double pivotCorectionAngl = robotOrientation - globalPositionUpdate.returnOrientation();
            double pivotCorectionPower = pivotCorectionAngl / 180;

            //slows down as it nears the target (average of the turn and distance error)
            double slowDown;
            double distanceSlowDown;
            double angleSlowDown;

            if (Math.abs(distance / COUNTS_PER_INCH) <= 3)
            {
                distanceSlowDown = Math.abs(distance / 3);
            }

            else
            {
                distanceSlowDown = 1;
            }

            if (Math.abs(robotOrientation) <= 5 && Math.abs(distance / COUNTS_PER_INCH) <= 3)
            {
                angleSlowDown = Math.abs(robotOrientation / 5);
            }

            else
            {
                angleSlowDown = 1;
            }

            slowDown = (angleSlowDown + distanceSlowDown) / 2;



            //sets the power of the motors
            double LFpower = (robotMovementXComponent + robotMovementYComponent + pivotCorectionPower) * slowDown * robotPower;
            double LBpower = (robotMovementXComponent - robotMovementYComponent + pivotCorectionPower) * slowDown * robotPower;
            double RFpower = (robotMovementXComponent - robotMovementYComponent - pivotCorectionPower) * slowDown * robotPower;
            double RBpower = (robotMovementXComponent + robotMovementYComponent - pivotCorectionPower) * slowDown * robotPower;

//if statement reduces/increases motor power accordingly if a motor has more than a power of 1 or less than a power of -1
//that way all the motors remain proportional but at the highest speed possible forward or reverse
//if you move slowly there is nothing to reduce and it will still go slowly

            double motorPowerRatio = 1;

            if (LFpower >= 1 && LFpower >= LBpower && LFpower >= RFpower && LFpower >= RBpower)
            {
                motorPowerRatio = 1 / LFpower;
            } else if (LBpower >= 1 && LBpower >= LFpower && LBpower >= RFpower && LBpower >= RBpower)
            {
                motorPowerRatio = 1 / LBpower;
            } else if (RFpower >= 1 && RFpower >= LFpower && RFpower >= LBpower && RFpower >= RBpower)
            {
                motorPowerRatio = 1 / RFpower;
            } else if (RBpower >= 1 && RBpower >= LFpower && RBpower >= RFpower && RBpower >= LBpower)
            {
                motorPowerRatio = 1 / RBpower;
            } else if (LFpower <= -1 && LFpower <= LBpower && LFpower <= RFpower && LFpower <= RBpower)
            {
                motorPowerRatio = -1 / LFpower;
            } else if (LBpower <= -1 && LBpower <= LFpower && LBpower <= RFpower && LBpower <= RBpower)
            {
                motorPowerRatio = -1 / LBpower;
            } else if (RFpower <= -1 && RFpower <= LFpower && RFpower <= LBpower && RFpower <= RBpower)
            {
                motorPowerRatio = -1 / RFpower;
            } else if (RBpower <= -1 && RBpower <= LFpower && RBpower <= RFpower && RBpower <= LBpower)
            {
                motorPowerRatio = -1 / RBpower;
            }



//robot power is your speed multiplier

            robot.motorRF.setPower(RFpower * motorPowerRatio);
            robot.motorRB.setPower(RBpower * motorPowerRatio);
            robot.motorLB.setPower(LBpower * motorPowerRatio);
            robot.motorLF.setPower(LFpower * motorPowerRatio);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("motorRF power", robot.motorRF.getPower());
            telemetry.addData("motorRB power", robot.motorRB.getPower());
            telemetry.addData("motorLF power", robot.motorLF.getPower());
            telemetry.addData("motorLB power", robot.motorLB.getPower());

            telemetry.update();


        }

        turnCorrection(robotPower, robotOrientation, allowableOrientationError);

        setPowerAll(0);
    }

    public void turnCorrection(double robotPower, double robotOrientation, double allowableOrientationError) {

        RobotHardware robot = new RobotHardware(hardwareMap);

        double pivotCorectionAngle = robotOrientation - globalPositionUpdate.returnOrientation();

        while(opModeIsActive() && Math.abs(pivotCorectionAngle) >= allowableOrientationError)
        {

            pivotCorectionAngle = robotOrientation - globalPositionUpdate.returnOrientation();
            double pivotCorectionPower = pivotCorectionAngle / 15;

            //slows down as it nears the target
            double slowDown;

            if (pivotCorectionAngle >= 5)
            {
                slowDown = Math.abs(pivotCorectionPower / 5);
            } else
            {
                slowDown = 1;
            }


            //sets the power of the motors
            double LFpower = (pivotCorectionPower) /* slowDown */ * robotPower;
            double LBpower = (pivotCorectionPower) /* slowDown */ * robotPower;
            double RFpower = (-pivotCorectionPower) /* slowDown */ * robotPower;
            double RBpower = (-pivotCorectionPower) /* slowDown */ * robotPower;

//if statement reduces/increases motor power accordingly if a motor has more than a power of 1 or less than a power of -1
//that way all the motors remain proportional but at the highest speed possible forward or reverse
//if you move slowly there is nothing to reduce and it will still go slowly

            double motorPowerRatio = 1;

            if (LFpower >= 1 && LFpower >= LBpower && LFpower >= RFpower && LFpower >= RBpower)
            {
                motorPowerRatio = 1 / LFpower;
            } else if (LBpower >= 1 && LBpower >= LFpower && LBpower >= RFpower && LBpower >= RBpower)
            {
                motorPowerRatio = 1 / LBpower;
            } else if (RFpower >= 1 && RFpower >= LFpower && RFpower >= LBpower && RFpower >= RBpower)
            {
                motorPowerRatio = 1 / RFpower;
            } else if (RBpower >= 1 && RBpower >= LFpower && RBpower >= RFpower && RBpower >= LBpower)
            {
                motorPowerRatio = 1 / RBpower;
            } else if (LFpower <= -1 && LFpower <= LBpower && LFpower <= RFpower && LFpower <= RBpower)
            {
                motorPowerRatio = -1 / LFpower;
            } else if (LBpower <= -1 && LBpower <= LFpower && LBpower <= RFpower && LBpower <= RBpower)
            {
                motorPowerRatio = -1 / LBpower;
            } else if (RFpower <= -1 && RFpower <= LFpower && RFpower <= LBpower && RFpower <= RBpower)
            {
                motorPowerRatio = -1 / RFpower;
            } else if (RBpower <= -1 && RBpower <= LFpower && RBpower <= RFpower && RBpower <= LBpower)
            {
                motorPowerRatio = -1 / RBpower;
            }


//robot power is your speed multiplier

            robot.motorRF.setPower(RFpower * motorPowerRatio);
            robot.motorRB.setPower(RBpower * motorPowerRatio);
            robot.motorLB.setPower(LBpower * motorPowerRatio);
            robot.motorLF.setPower(LFpower * motorPowerRatio);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontal.getCurrentPosition());

            telemetry.addData("motorRF power", robot.motorRF.getPower());
            telemetry.addData("motorRB power", robot.motorRB.getPower());
            telemetry.addData("motorLF power", robot.motorLF.getPower());
            telemetry.addData("motorLB power", robot.motorLB.getPower());

            telemetry.update();
        }
    }

    public void setPowerAll(double power)
    {
        RobotHardware robot = new RobotHardware(hardwareMap);

        robot.motorRF.setPower(power);
        robot.motorRB.setPower(power);
        robot.motorLB.setPower(power);
        robot.motorLF.setPower(power);
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