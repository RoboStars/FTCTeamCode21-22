

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class testbotdjaoiwejfae {
    public DcMotor ballintake = null;
    public DcMotor ballintake2 = null;
    public DcMotor linearactuator1 = null;
    public Servo horizontalservo = null;
    public Servo verticalservo = null;

    HardwareMap hwMap = null;
    //InitIMU imuInit            = null;
    private ElapsedTime period = new ElapsedTime();


    public testbotdjaoiwejfae() {

    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        linearactuator1 = hwMap.dcMotor.get("linearactuator");
        ballintake = hwMap.dcMotor.get("frontL");
        ballintake2 = hwMap.dcMotor.get("backL");
        horizontalservo = hwMap.servo.get("servohorizontallinear");
        verticalservo = hwMap.servo.get("servoverticallinear");
    }
}