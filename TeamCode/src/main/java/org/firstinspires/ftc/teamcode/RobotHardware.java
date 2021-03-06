package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by User on 9/26/2017.
 */

public class RobotHardware
{
    //drive motors
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;
    
    //mechanism motors
    public DcMotorEx flywheel = null;
    public DcMotor wobble = null;
    public DcMotorEx intake1 = null;
    public DcMotor intake2 = null;

    //odometry encoders
    public DcMotor verticalRight = null;
    public DcMotor verticalLeft = null;
    public DcMotor horizontal = null;
    
    //servos
    public Servo servo = null;
    public Servo servo2 = null;

    boolean useEncoder = false;

    HardwareMap hwMap = null;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public RobotHardware(HardwareMap ahwMap)
    {
        //dive motors
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");
        
        //mechanism motors
        flywheel = ahwMap.get(DcMotorEx.class, "flywheel");
        wobble = ahwMap.dcMotor.get("wobble");
        intake1 = ahwMap.get(DcMotorEx.class, "intake1");
        intake2 = ahwMap.dcMotor.get("intake2");

        //odometry encoders
        //verticalRight = motorLF;
        //verticalLeft = motorRF;
        //horizontal = motorRB;

        //servos
        servo = ahwMap.servo.get("servo");
        servo2 = ahwMap.servo.get("servo2");
        
        //drive motors and odometry encoders
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        

        
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);
        
        //mechanism motors
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobble.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setPower(0);
        wobble.setPower(0);
        intake1.setPower(0);
        intake1.setPower(0);
        
        //servos
        servo.setPosition(0.25);
        servo2.setPosition(0);
    }

}
