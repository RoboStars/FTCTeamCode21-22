package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name="freight", group="TestBot")

public class teleopfreight extends OpMode {
    HardwareTestbot robot = new HardwareTestbot();

    PController pController = new PController(0.03);



    double x1 = 0; // left
    double y1 = 0; // right
    double fortyFiveInRads = -Math.PI/4;
    double cosine45 = Math.cos(fortyFiveInRads);
    double sine45 = Math.sin(fortyFiveInRads);
    double x2 = 0;
    double y2 = 0;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //pid code
        int encoderDegreesToAttain = 20000; //filler value
        double minPower = 0.01;
        double maxPower = 0.5;
        pController.setInputRange(50, 500);
        pController.setSetPoint(encoderDegreesToAttain);
        pController.setOutputRange(minPower, maxPower);

        double spin = gamepad1.right_stick_x;

        if(Math.abs(spin)>0.1){

            robot.leftFront.setPower(spin);
            robot.leftBack.setPower(spin);
            robot.rightFront.setPower(-spin);
            robot.rightBack.setPower(-spin);

        } else {

            y1 = -gamepad1.left_stick_y;
            x1 =  gamepad1.left_stick_x;

            //need to rotate 45 degrees
            y2 = y1*cosine45 + x1*sine45;
            x2 = x1*cosine45 - y1*sine45;

            robot.leftFront.setPower(x2);
            robot.leftBack.setPower(y2);
            robot.rightFront.setPower(y2);
            robot.rightBack.setPower(x2);
        }


        telemetry.addData("x1", "%.2f", x1);
        telemetry.addData("y1", "%.2f", y1);
        telemetry.addData("x2", "%.2f", x2);
        telemetry.addData("y2", "%.2f", y2);

        if(gamepad2.a){
            robot.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.carousel.setPower(0.3);
            robot.carousel.setTargetPosition(140);


        }
        //arm code using PID
        if(gamepad2.b){

            //move arm to low level, need to add encoder value
            while (encoderDegreesToAttain != robot.arm.getCurrentPosition()) {
                telemetry.addData("encoder position", robot.arm.getCurrentPosition());
                telemetry.addData("power", pController.getComputedOutput(robot.arm.getCurrentPosition()));
                telemetry.update();
                if (robot.arm.getCurrentPosition() < encoderDegreesToAttain) {
                    robot.arm.setPower(minPower + pController.getComputedOutput(robot.arm.getCurrentPosition()));
                } else {
                    robot.arm.setPower(minPower - pController.getComputedOutput(robot.arm.getCurrentPosition()));
                }

                if(robot.arm.getCurrentPosition()>=34500 || (robot.arm.getCurrentPosition()<=35500)){
                    robot.arm.setPower(0);
                }
            }
        }

        if(gamepad2.x){
            //move arm to mid level, need to add encoder value
            while (encoderDegreesToAttain != robot.arm.getCurrentPosition()) {
                telemetry.addData("encoder position", robot.arm.getCurrentPosition());
                telemetry.addData("power", pController.getComputedOutput(robot.arm.getCurrentPosition()));
                telemetry.update();
                if (robot.arm.getCurrentPosition() < encoderDegreesToAttain) {
                    robot.arm.setPower(minPower + pController.getComputedOutput(robot.arm.getCurrentPosition()));
                } else {
                    robot.arm.setPower(minPower - pController.getComputedOutput(robot.arm.getCurrentPosition()));
                }

                if(robot.arm.getCurrentPosition()>=34500 || (robot.arm.getCurrentPosition()<=35500)){
                    robot.arm.setPower(0);
                }
            }
        }

        if(gamepad2.y){
            //move arm to high level, need to set encoder value
            while (encoderDegreesToAttain != robot.arm.getCurrentPosition()) {
                telemetry.addData("encoder position", robot.arm.getCurrentPosition());
                telemetry.addData("power", pController.getComputedOutput(robot.arm.getCurrentPosition()));
                telemetry.update();
                if (robot.arm.getCurrentPosition() < encoderDegreesToAttain) {
                    robot.arm.setPower(minPower + pController.getComputedOutput(robot.arm.getCurrentPosition()));
                } else {
                    robot.arm.setPower(minPower - pController.getComputedOutput(robot.arm.getCurrentPosition()));
                }

                if(robot.arm.getCurrentPosition()>=34500 || (robot.arm.getCurrentPosition()<=35500)){
                    robot.arm.setPower(0);
                }
            }
        }

        if(gamepad2.dpad_up){
            //move arm to top, need to set encoder value
            while (encoderDegreesToAttain != robot.arm.getCurrentPosition()) {
                telemetry.addData("encoder position", robot.arm.getCurrentPosition());
                telemetry.addData("power", pController.getComputedOutput(robot.arm.getCurrentPosition()));
                telemetry.update();
                if (robot.arm.getCurrentPosition() < encoderDegreesToAttain) {
                    robot.arm.setPower(minPower + pController.getComputedOutput(robot.arm.getCurrentPosition()));
                } else {
                    robot.arm.setPower(minPower - pController.getComputedOutput(robot.arm.getCurrentPosition()));
                }

                if(robot.arm.getCurrentPosition()>=34500 || (robot.arm.getCurrentPosition()<=35500)){
                    robot.arm.setPower(0);
                }
            }
        }








        double up;
        double down;




    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}