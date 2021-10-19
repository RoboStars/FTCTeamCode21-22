package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

@TeleOp(name="teleop freight", group="")

public class teleopfreight extends OpMode {

    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    double armPosition = 0.75;
    final double ARM_SPEED = 0.02;
    double servoPosition = 0.25;
    final double SERVO_SPEED = 0.02;
    double armencoder;
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
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armencoder = robot.arm.getCurrentPosition();
        // robot.Arm.setPosition(0.5); //setPosition actually sets the servo's position and moves it
        // robot.Gripper.setPosition(0.5);
        // robot.long_arm.setPosition(0.5);
        // robot.big_gripper.setPosition(0.5);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */



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

        if(gamepad1.a){
            robot.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.carousel.setTargetPosition(138);
            robot.carousel.setPower(3);
            if(robot.carousel.getCurrentPosition()>138 && robot.carousel.getCurrentPosition()<140) {
                robot.carousel.setPower(0);
            }
        }

        if(gamepad1.b){
            //arm code
        }








    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
