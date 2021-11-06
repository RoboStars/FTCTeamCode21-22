package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TestBot: 18inches", group="TestBot")

public class Inches_18 extends LinearOpMode {
    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    // todo: write your code here

    double A, B, C, D;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        A = robot.leftFront.getCurrentPosition();
        B = robot.rightFront.getCurrentPosition();
        C = robot.leftBack.getCurrentPosition();
        D = robot.rightBack.getCurrentPosition();

        telemetry.addData("left front", "%.2f", A);
        telemetry.addData("right front", "%.2f", B);
        telemetry.addData("left back", "%.2f", C);
        telemetry.addData("right back", "%.2f", D);
        telemetry.update();
        sleep(1000);
        // prints uninitialized value

        // robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double CIRCUMFERENCE = 3.14*4;
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//sets the counter of ticks to 0

        A = robot.leftFront.getCurrentPosition();
        B = robot.rightFront.getCurrentPosition();
        C = robot.leftBack.getCurrentPosition();
        D = robot.rightBack.getCurrentPosition();

        telemetry.addData("left front", "%.2f", A);
        telemetry.addData("right front", "%.2f", B);
        telemetry.addData("left back", "%.2f", C);
        telemetry.addData("right back", "%.2f", D);
        telemetry.update();
        sleep(1000);
        //print zero

        double rotationsNeeded = 17.5/CIRCUMFERENCE;
        int encoderDrivingTarget = (int) (rotationsNeeded*robot.ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);

        robot.leftFront.setTargetPosition(encoderDrivingTarget);
        robot.leftBack.setTargetPosition(encoderDrivingTarget);
        robot.rightFront.setTargetPosition(encoderDrivingTarget);
        robot.rightBack.setTargetPosition(encoderDrivingTarget);

        robot.leftFront.setPower(robot.POINT_FOUR);
        robot.leftBack.setPower(robot.POINT_FOUR);
        robot.rightFront.setPower(robot.POINT_FOUR);
        robot.rightBack.setPower(robot.POINT_FOUR);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(robot.leftFront.isBusy() || robot.rightFront.isBusy()){

        }
        A = robot.leftFront.getCurrentPosition();
        B = robot.rightFront.getCurrentPosition();
        C = robot.leftBack.getCurrentPosition();
        D = robot.rightBack.getCurrentPosition();

        telemetry.addData("left front encoder", "%.2f", A);
        telemetry.addData("right front encoder", "%.2f", B);
        telemetry.addData("left back encoder", "%.2f", C);
        telemetry.addData("right back encoder", "%.2f", D);
        sleep(2000);

        // print encoder value


        if(robot.leftFront.getCurrentPosition() >= robot.leftFront.getTargetPosition()) {

            A = robot.leftFront.getCurrentPosition();
            B = robot.rightFront.getCurrentPosition();
            C = robot.leftBack.getCurrentPosition();
            D = robot.rightBack.getCurrentPosition();

            telemetry.addData("left front encoder", "%.2f", A);
            telemetry.addData("right front encoder", "%.2f", B);
            telemetry.addData("left back encoder", "%.2f", C);
            telemetry.addData("right back encoder", "%.2f", D);
            //telemetry.addData("error", "%.2f", error);
            telemetry.update();

            sleep (5000);

            robot.leftFront.setPower(robot.ZERO);
            robot.leftBack.setPower(robot.ZERO);
            robot.rightFront.setPower(robot.ZERO);
            robot.rightBack.setPower(robot.ZERO);

            sleep(1000);

            robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // A = robot.leftFront.getCurrentPosition();
            // B = robot.rightFront.getCurrentPosition();
            // C = robot.leftBack.getCurrentPosition();
            // D = robot.rightBack.getCurrentPosition();

            // telemetry.addData("left front encoder", "%.2f", A);
            // telemetry.addData("right front encoder", "%.2f", B);
            // telemetry.addData("left back encoder", "%.2f", C);
            // telemetry.addData("right back encoder", "%.2f", D);
            // //telemetry.addData("error", "%.2f", error);
            // telemetry.update();
            // sleep(3000);
        }




        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
