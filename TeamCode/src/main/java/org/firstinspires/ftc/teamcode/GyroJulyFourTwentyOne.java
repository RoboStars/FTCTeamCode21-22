package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name = "IMUTest 4th 2021", group = "TestBot")

public class GyroJulyFourTwentyOne extends LinearOpMode {
    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        robot.imu.initialize(parameters);

        waitForStart();

        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double leftPower = robot.POINT_SIX;
        double rightPower = robot.POINT_SIX;

        robot.leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        int TargetPosition = 2226;
        int targetAngle = 0;

        robot.leftFront.setTargetPosition(TargetPosition);
        robot.leftBack.setTargetPosition(TargetPosition);
        robot.rightFront.setTargetPosition(TargetPosition);
        robot.rightBack.setTargetPosition(TargetPosition);

        robot.leftFront.setPower(leftPower);
        robot.leftBack.setPower(leftPower);
        robot.rightFront.setPower(rightPower);
        robot.rightBack.setPower(rightPower);
        while(robot.leftFront.getCurrentPosition() < TargetPosition) {
            if(robot.angles.firstAngle < targetAngle) {
                rightPower += 0.05;
                leftPower -= 0.05;
            }
            else if(robot.angles.firstAngle > targetAngle){
                leftPower += 0.05;
                rightPower -= 0.05;
            }
            else{
                leftPower = robot.POINT_SIX;
                rightPower = robot.POINT_SIX;
            }
        }

        leftPower = 0;
        rightPower = 0;
        robot.zero();




    }
}
