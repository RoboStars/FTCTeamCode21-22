
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class testbotdjaoiwejfae {
    public DcMotor ballintake = null;
    public DcMotor ballintake2 = null;
    public DcMotor linearactuator1 = null;

    HardwareMap hwMap           =  null;
    //InitIMU imuInit            = null;
    private ElapsedTime period  = new ElapsedTime();


    public testbotdjaoiwejfae(){

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        ballintake  = hwMap.dcMotor.get("frontL");
        ballintake2  = hwMap.dcMotor.get("backL");

        ballintake.setDirection(DcMotor.Direction.REVERSE);
        ballintake2.setDirection(DcMotor.Direction.REVERSE);

        ballintake.setPower(0);
        ballintake2.setPower(0);

        ballintake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballintake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }


}
