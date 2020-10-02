package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by User on 9/26/2017.
 */

public class RobotHardware
{
    public DcMotor motorRF = null;
    public DcMotor motorLF = null;
    public DcMotor motorRB = null;
    public DcMotor motorLB = null;

    boolean useEncoder = false;

    HardwareMap hwMap = null;
    private ElapsedTime elapsedTime = new ElapsedTime();

    public RobotHardware(HardwareMap ahwMap)
    {
        init(ahwMap);
    }

    public void init(HardwareMap ahwMap)
    {
        motorRF = ahwMap.dcMotor.get("motorRF");
        motorLF = ahwMap.dcMotor.get("motorLF");
        motorRB = ahwMap.dcMotor.get("motorRB");
        motorLB = ahwMap.dcMotor.get("motorLB");

        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRF.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorLB.setPower(0);

        //

    }
}
