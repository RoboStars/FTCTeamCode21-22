package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    //Drive motors

    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double TICKS_PER_INCH = robot.ONEONEFIVEZERO_MOTOR_TICK_COUNT/robot.CIRCUMFERENCE*2;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "frontR", rbName = "backR", lfName = "frontL", lbName = "backL";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;



    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, TICKS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();



        while(opModeIsActive()){

            if(gamepad1.a) {
                horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                horizontal.setTargetPosition(-20);


                while(horizontal.getCurrentPosition()>horizontal.getTargetPosition()){
                    robot.strafe_left(0.3);
                    telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                    telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                    telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
                }

                robot.zero();
                telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
                telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            }

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / TICKS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / TICKS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    private void initDriveHardwareMap(String vlEncoderName, String vrEncoderName, String hEncoderName){


        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);



        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public void gotoPosition(double targetX, double targetY, double power, double finalOrientation, double allowableDistanceError)
    {
        targetX = targetX * TICKS_PER_INCH;
        targetY = targetY * TICKS_PER_INCH;

        double distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        allowableDistanceError = allowableDistanceError * TICKS_PER_INCH;

        while (opModeIsActive() && (distance > allowableDistanceError)) {
            distanceToXTarget = targetX - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetY - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));
            double robotMovmentXComponent = calculateX(robotMovementAngle, power);
            double robotMovmentYComponent = calculateY(robotMovementAngle, power);

            double pivotCorrection = finalOrientation - globalPositionUpdate.returnOrientation();

            moveHolonomic(robotMovmentYComponent,robotMovmentXComponent,pivotCorrection);

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        }
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / TICKS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / TICKS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
        telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
        telemetry.update();


    }

    public void moveHolonomic(double x, double y , double z)
    {
        double max_power = 0.4;
        double min_power = -1*max_power;

        double fl_power = Range.clip(y + x - z, min_power, max_power);
        double fr_power = Range.clip(y - x + z, min_power, max_power);
        double br_power = Range.clip(y + x + z, min_power, max_power);
        double bl_power = Range.clip(y - x - z, min_power, max_power);

        // Sets the power of the motors to the power defined above

        robot.leftFront.setPower(fl_power);
        robot.leftBack.setPower(bl_power);
        robot.rightFront.setPower(fr_power);
        robot.rightBack.setPower(br_power);


    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}