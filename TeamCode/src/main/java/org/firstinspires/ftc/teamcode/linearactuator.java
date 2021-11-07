package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;




/*
Logic for the encoder incremental movement for Sat.


 */


@TeleOp(name="linear actuator", group="TestBot")
//@Disabled
public class linearactuator extends LinearOpMode {
    testbotdjaoiwejfae robot = new testbotdjaoiwejfae();


    //HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
   // private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int encoderDegreesToAttain = 1000;
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.03);
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        robot.linearactuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearactuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();



        while(opModeIsActive()) {

            double left;
            double right;

             // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            robot.linearactuator1.setPower(left);

            // Send telemetry message to signify robot running;
            telemetry.addData("position", robot.linearactuator1.getCurrentPosition());
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            if(gamepad1.a){
                robot.verticalservo.setPosition(robot.verticalservo.getPosition()+0.05);
            }

            if(gamepad1.b){
                robot.verticalservo.setPosition(robot.verticalservo.getPosition()-0.05);
            }

            if(gamepad1.x){
                robot.horizontalservo.setPosition(robot.horizontalservo.getPosition()+0.05);
            }

            if(gamepad1.y){
                robot.horizontalservo.setPosition(robot.horizontalservo.getPosition()-0.05);
            }





        }
    }
}




