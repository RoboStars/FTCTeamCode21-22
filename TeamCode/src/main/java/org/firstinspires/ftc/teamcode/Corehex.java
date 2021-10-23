package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;





/*
Logic for the encoder incremental movement for Sat.


 */


@TeleOp(name="linear actuator", group="TestBot")
//@Disabled
public class Corehex extends LinearOpMode {



    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    ElapsedTime runtime = new ElapsedTime();

    public DcMotor corehex;




    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        corehex = hardwareMap.dcMotor.get("corehex");
        int encoderDegreesToAttain = 1000;
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.03);
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);


        waitForStart();

        runhex(1, 0.4);
        telemetry.addData("encoder position", corehex.getCurrentPosition());
        telemetry.update();
        telemetry.setAutoClear(false);



    }

    public void runhex(double rotations, double power ){
        double rotation_degree = 288;

        int degrees = (int)(rotation_degree*rotations);
        corehex.setTargetPosition(degrees);
        corehex.setPower(power);
        corehex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        corehex.setPower(0);




    }
}




