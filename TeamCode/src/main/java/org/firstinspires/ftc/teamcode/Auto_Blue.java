package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import android.app.Activity;
import com.qualcomm.robotcore.util.Hardware;
import android.graphics.Color;
import android.view.View;
import java.util.Locale;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

@Autonomous(name="auto blue", group="TestBot")

public class Auto_Blue  extends OpMode {
    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    private enum State {
        CODE_1,
        CODE_2,
        CODE_3,
        MOVE_TO_HUB,
        MOVE_TO_CAROUSEL,
        SPIN_CAROUSEL,
        COME_TO_WAREHOUSE


    }

    private State currentState;

    private void newState(State newState){
        runtime.reset();
        currentState = newState;
    }



    @Override
    public void init() {
        robot.init(hardwareMap);



        robot.zero();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void init_loop(){

        telemetry.addData("Status", "ready to run");
        telemetry.update();
    }

    @Override
    public void start(){
        newState(State.CODE_2);

    }

    @Override
    public void loop(){
        switch (currentState){
            case CODE_2:
                newState(State.MOVE_TO_HUB);

                break;
            case CODE_1:
                //turn robot to see code one,
                //if sensed, straighten and go to case MOVE TO SHIPPING HUB
                //else, go to case 3
               break;
            case CODE_3:
                //move arm up to highest and go to shipping to shipping hub
                break;
            case MOVE_TO_HUB:
                robot.reset_encoder();
                while(robot.rightFront.getCurrentPosition()<570) {
                    robot.strafe_left(0.3);
                }
                robot.zero();
                newState(State.MOVE_TO_CAROUSEL);
                break;
            case MOVE_TO_CAROUSEL:

                while(robot.leftFront.getCurrentPosition()<200) {
                    robot.leftFront.setPower(0.3);
                    robot.leftBack.setPower(0.3);
                }
                robot.zero();
                robot.Move_Forward(32,0.5);
                newState(State.SPIN_CAROUSEL);

                break;
            case SPIN_CAROUSEL:
                robot.carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //resetting encoders to make sure there are no issues


                robot.carousel.setTargetPosition(138);
                //setting the  target position to the value we found using formula

                robot.carousel.setPower(0.25);
                //setting power


                robot.carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //setting the mode to run to position so that the encoders work

                robot.carousel.setPower(0);


                if(robot.carousel.isBusy()){
                    while(robot.carousel.getCurrentPosition() < 138 && robot.carousel.getCurrentPosition() < 140){
                        telemetry.addData("Encoder", "Motors have rotated carousel once");
                        telemetry.update();
                        //setting telemetry telling us when the carousel has rotated once
                    }

                }

                break;
            case COME_TO_WAREHOUSE:
                //go to warehouse, makesure robot is completely in warehouse



        }
        telemetry.addData("Status", "out of loop");
        telemetry.update();

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        telemetry.addData("Status", "in stop");
        telemetry.update();
    }


}



