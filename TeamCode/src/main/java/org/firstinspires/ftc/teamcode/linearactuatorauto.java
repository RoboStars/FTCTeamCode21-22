package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "linear actuator auto", group = "Testbot")

public class linearactuatorauto extends LinearOpMode {
    testbotdjaoiwejfae robot = new testbotdjaoiwejfae();// Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int encoderDegreesToAttain = 20000;
        double minPower = 0.01;
        double maxPower = 0.5;
        PController pController = new PController(0.03);
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        robot.linearactuator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearactuator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                while (encoderDegreesToAttain != robot.linearactuator1.getCurrentPosition()) {
                    telemetry.addData("encoder position", robot.linearactuator1.getCurrentPosition());
                    telemetry.addData("power", pController.getComputedOutput(robot.linearactuator1.getCurrentPosition()));
                    telemetry.update();
                    if (robot.linearactuator1.getCurrentPosition() < encoderDegreesToAttain) {
                        robot.linearactuator1.setPower(minPower + pController.getComputedOutput(robot.linearactuator1.getCurrentPosition()));
                    } else {
                        robot.linearactuator1.setPower(minPower - pController.getComputedOutput(robot.linearactuator1.getCurrentPosition()));
                    }
                }

            }

            if(gamepad1.a){
                encoderDegreesToAttain = 35000;
                while (encoderDegreesToAttain != robot.linearactuator1.getCurrentPosition()) {
                    telemetry.addData("encoder position", robot.linearactuator1.getCurrentPosition());
                    telemetry.addData("power", pController.getComputedOutput(robot.linearactuator1.getCurrentPosition()));
                    telemetry.update();
                    if (robot.linearactuator1.getCurrentPosition() < encoderDegreesToAttain) {
                        robot.linearactuator1.setPower(minPower + pController.getComputedOutput(robot.linearactuator1.getCurrentPosition()));
                    } else {
                        robot.linearactuator1.setPower(minPower - pController.getComputedOutput(robot.linearactuator1.getCurrentPosition()));
                    }

                    if(robot.linearactuator1.getCurrentPosition()>=34500 || (robot.linearactuator1.getCurrentPosition()<=35500)){
                        robot.linearactuator1.setPower(0);
                    }
                }
            }
        }








    }
}