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

@Autonomous(name="auto blue (left)", group="TestBot") //Aarav Jagtiani, Rishabh Mahesh
// simple auto
// robot senses barcode, drops preload element in correct level, spins duck off carousel, parks in storage unit
public class Auto_Blue_Left extends OpMode {
    HardwareTestbot         robot   = new HardwareTestbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    // all states
    private enum State {
        CODE_1, // barcode 1
        CODE_2, // barcode 2
        CODE_3, // barcode 3
        MOVE_TO_HUB, //robot drives to shipping hub
        DRIVE_TO_WAREHOUSE //robot drives to warehouse


    }

    private State currentState;

    private void newState(State newState){
        runtime.reset();
        currentState = newState;
    }



    @Override
    public void init() {

        //initialize hardware map
        robot.init(hardwareMap);


        //power off motors
        robot.zero();

        //reset encoders
        robot.reset_encoder();


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

                //assuming robot senses 2 as of right now 10/12/21
                newState(State.MOVE_TO_HUB);

                break;
            case CODE_1:
                //turn robot to see code one,
                //if sensed, straighten and go to case MOVE TO SHIPPING HUB
                //else, go to case 3
                break;
            case CODE_3:
                //assumes element is at code 3
                //move arm up to highest and go to shipping to shipping hub
                break;
            case MOVE_TO_HUB:

                //reset encoders
                robot.reset_encoder();

                //strafe for 570 positions
                while(robot.rightFront.getCurrentPosition()<570) {
                    robot.strafe_left(0.3);
                }

                //stop motors
                robot.zero();

                //next state
                newState(State.DRIVE_TO_WAREHOUSE);
                break;
            case DRIVE_TO_WAREHOUSE:
                //turn robot right for 200 encoder positions
                while(robot.leftFront.getCurrentPosition()<200) {
                    robot.leftFront.setPower(0.3);
                    robot.leftBack.setPower(0.3);
                }
                //power off motors
                robot.zero();
                //moves toward carousel
                robot.Move_Forward(72,0.5);
                // next state


                break;



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



