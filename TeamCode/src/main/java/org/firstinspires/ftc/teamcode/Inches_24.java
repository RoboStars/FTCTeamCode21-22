package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="TestBot: 18inches", group="TestBot")

public class Inches_24 extends LinearOpMode {
    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    // todo: write your code here

    double A, B, C, D;
    double base_degrees;
    double last_time;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".4
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        telemetry.addData("Gyro Initializiation Status: ", imu.isGyroCalibrated());
        telemetry.update();
        try {
            sleep(50);
        } catch(Exception e) { }
        while(!imu.isGyroCalibrated()) {
            telemetry.addData("Gyro Initializiation Status: ", imu.isGyroCalibrated());
            telemetry.update();
            try {
                sleep(50);
            } catch (Exception e) {}
        }

        telemetry.addData("Gyro Initializiation Status: ", imu.isGyroCalibrated());
        telemetry.addData("Current Gyro Heading: ", robot.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        base_degrees = robot.getHeading(AngleUnit.DEGREES);
        last_time = getRuntime();

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

        double rotationsNeeded = 23.5/CIRCUMFERENCE;
        int encoderDrivingTarget = (int) (rotationsNeeded*robot.ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);

        while (robot.leftFront.getCurrentPosition() <= encoderDrivingTarget){
            if (robot.getHeading(AngleUnit.DEGREES) > -1 && robot.getHeading(AngleUnit.DEGREES) < 1) {
                robot.forward(0.1);
                telemetry.addData("Current State: ", "Straight");
                telemetry.update();
            } else {
                if (robot.getHeading(AngleUnit.DEGREES) < -1) {
                    robot.turn_right(0.5);
                    telemetry.addData("Current State: ", "Turning Left");
                    telemetry.addData("Current Angle: ", robot.getHeading(AngleUnit.DEGREES));
                    telemetry.update();

                }
                if (robot.getHeading(AngleUnit.DEGREES) > 1) {
                    robot.turn_left(0.5);
                    telemetry.addData("Current Angle: ", robot.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("Current State: ", "Turning Right");
                    telemetry.update();
                }
            }

            telemetry.addData("left front", robot.leftFront.getCurrentPosition());
            telemetry.addData("right front", robot.rightFront.getCurrentPosition());
            telemetry.addData("left back", robot.leftBack.getCurrentPosition());
            telemetry.addData("right back", robot.rightBack.getCurrentPosition());
            telemetry.update();


        }

        robot.zero();

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
