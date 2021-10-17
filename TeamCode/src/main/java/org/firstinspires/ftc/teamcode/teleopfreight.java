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

@TeleOp(name="TestBot: Test3", group="TestBot")

public class teleopfreight extends OpMode {
    HardwareTestbot robot = new HardwareTestbot();
//encoder 138-140

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
        double left;
        double right;

        // // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftFront.setPower(left);
        robot.leftBack.setPower(left);
        robot.rightFront.setPower(right);
        robot.rightBack.setPower(right);

        //Strafe Left
        if(gamepad1.left_bumper){
            robot.leftFront.setPower(-robot.ONE);
            robot.leftBack.setPower(robot.ONE);
            robot.rightFront.setPower(robot.ONE);
            robot.rightBack.setPower(-robot.ONE);
        }


        if(gamepad1.a){
            robot.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.carousel.setPower(0.3);
            robot.carousel.setTargetPosition(140);


        }

        if(gamepad1.b){
            robot.arm.setPower(0.3);
        }


        //Strafe Right
        if(gamepad1.right_bumper){
            robot.leftFront.setPower(robot.ONE);
            robot.leftBack.setPower(-robot.ONE);
            robot.rightFront.setPower(-robot.ONE);
            robot.rightBack.setPower(robot.ONE);
        }



        double up;
        double down;


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}