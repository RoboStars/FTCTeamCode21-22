package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Map;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


public class HardwareTestbot
{
    /* Public OpMode members. */
    public DcMotor  leftFront = null;
    public DcMotor  leftBack  = null;
    public DcMotor  rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor arm = null;
    public DcMotor carousel = null;
//    public Servo pusher = null;
//    public Servo gripper = null;
//    public ModernRoboticsI2cColorSensor color_sensor = null;
//    public ModernRoboticsI2cRangeSensor rangeSide = null;
//    public ModernRoboticsI2cRangeSensor rangeLeft = null;
//    public ModernRoboticsI2cRangeSensor rangeFront = null;
//    public ModernRoboticsI2cRangeSensor rangeBack = null;
//    public ModernRoboticsI2cGyro MR_Gyro = null;

    public float zOrientation;
    public static double globalAngle;
    public static final double ONEONEFIVEZERO_MOTOR_TICK_COUNT = 145.6;
    public static final double THREEONETWO_MOTOR_TICK_COUNT = 537.6;
    public static final double TWOTWOTHREE_MOTOR_TICK_COUNT = 753.6;
    public static final double ONEONESEVEN_MOTOR_TICK_COUNT = 1425.2;
    public static final double CIRCUMFERENCE = 3.14*3.93701;
    public static final double ZERO = 0.000;
    public static final double POINT_ONE = 0.1;
    public static final double POINT_TWO = 0.2;
    public static final double POINT_THREE = 0.3;
    public static final double POINT_FOUR = 0.4;
    public static final double POINT_FIVE = 0.5;
    public static final double POINT_SIX = 0.6;
    public static final double POINT_SEVEN = 0.7;
    public static final double POINT_EIGHT = 0.8;
    public static final double POINT_NINE = 0.9;
    public static final double ONE = 1;
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public final static double ARM_HOME = 0.0; // Starting position
    public final static double ARM_MIN_RANGE = 0.0; // Smallest number value allowed for servo position
    public final static double ARM_MAX_RANGE = 1.0; // Larget number value allowed for servo position
    public ModernRoboticsI2cGyro MR_Gyro = null;
    private BNO055IMU imu;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    //InitIMU imuInit            = null;
    private ElapsedTime period  = new ElapsedTime();
    private Orientation lastAngles = new Orientation();

    /* Constructor */
    public HardwareTestbot(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        leftFront  = hwMap.dcMotor.get("frontL");
        leftBack  = hwMap.dcMotor.get("backL");
        rightFront = hwMap.dcMotor.get("frontR");
        rightBack = hwMap.dcMotor.get("backR");
        arm = hwMap.dcMotor.get("arm");
        carousel = hwMap.dcMotor.get("carousel");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);




        // Set all motors to zero power
        leftFront.setPower(ZERO);
        leftBack.setPower(ZERO);
        rightFront.setPower(ZERO);
        rightBack.setPower(ZERO);
        arm.setPower(ZERO);
        carousel.setPower(ZERO);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);




//        color_sensor = hwMap.get(ModernRoboticsI2cColorSensor.class, "MR_color_sensor");
//        rangeSide = hwMap.get(ModernRoboticsI2cRangeSensor.class, "MR_range_side");
//        rangeLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "MR_range_left");
//        rangeFront = hwMap.get(ModernRoboticsI2cRangeSensor.class, "MR_range_front");
//        rangeBack = hwMap.get(ModernRoboticsI2cRangeSensor.class, "MR_range_back");
//        MR_Gyro = hwMap.get(ModernRoboticsI2cGyro.class, "MR_Gyro");
//
//        //MR_Gyro = (ModernRoboticsI2cGyro) sensorGyro;
//
//        // Define and Initialize Motors
//        gripper  = hwMap.servo.get("gripper");
//        pusher = hwMap.servo.get("pusha");
//
//        pusher.setDirection(Servo.Direction.REVERSE);


//setPosition actually sets the servo's position and moves it
//        gripper.setPosition(0);
//        pusher.setPosition(0.41);







        //// BNO055IMU.Parameters IMUparameters;

        // Create a new IMU parameters object
        //// IMUparameters = new BNO055IMU.Parameters();
        // Set the IMU mode to IMU so it automatically calibrates itself
        //// IMUparameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as our angle unit
        //// IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Use meters per second as a unit of accelerat
        //// IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver this may take several seconds!
        // telemetry.addData("Status", "Init IMU...please wait");
        // telemetry.update();
        // Intialize IMU using these parameters
        //// imu.initialize(IMUparameters);
        // Tell drive that init is done
        // telemetry.addData("Status", "IMU initialized");
        // telemetry.update();

        //Init_IMU();
    }

    public void init_IMU() {

        BNO055IMU.Parameters IMUparameters;

        // Create a new IMU parameters object
        IMUparameters = new BNO055IMU.Parameters();
        // Set the IMU mode to IMU so it automatically calibrates itself
        IMUparameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as our angle unit
        IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Use meters per second as a unit of acceleration
        IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver this may take several seconds!
        // telemetry.addData("Status", "Init IMU...please wait");
        // telemetry.update();
        // Intialize IMU using these parameters
        // Tell drive that init is done
        // telemetry.addData("Status", "IMU initialized");
        // telemetry.update();
    }

    //public void init(InitIMU imuInit) {
    /* code */

    //InitIMU = imuInit;
    BNO055IMU.Parameters IMUparameters;

    // // Create a new IMU parameters object
    // IMUparameters = new BNO055IMU.Parameters();
    // // Set the IMU mode to IMU so it automatically calibrates itself
    // IMUparameters.mode = BNO055IMU.SensorMode.IMU;
    // // Use degrees as our angle unit
    // IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    // // Use meters per second as a unit of acceleration
    // IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // // Warn driver this may take several seconds!
    // // telemetry.addData("Status", "Init IMU...please wait");
    // // telemetry.update();
    // // Intialize IMU using these parameters
    // imu.initialize(IMUparameters);
    // // Tell drive that init is done
    // // telemetry.addData("Status", "IMU initialized");
    // // telemetry.update();

    // Init_IMU();

    // Get the Z axis orientation of the IMU

//    public void gyro_leftstrafe(double speed){
//
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        // leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        // rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        // rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//
//        // double rotationsNeeded = inches/CIRCUMFERENCE;
//        // int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);
//
//        int zAngle = MR_Gyro.getIntegratedZValue();
//
//        // leftFront.setTargetPosition(-encoderDrivingTarget);
//        // leftBack.setTargetPosition(encoderDrivingTarget);
//        // rightFront.setTargetPosition(encoderDrivingTarget);
//        // rightBack.setTargetPosition(-encoderDrivingTarget);
//
//        leftFront.setPower(-speed);
//        leftBack.setPower(speed);
//        rightFront.setPower(speed);
//        rightBack.setPower(-speed);
//
//        // leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(leftFront.isBusy() || rightFront.isBusy()){
//            if(zAngle == 0){
//                leftFront.setPower(-speed);
//                leftBack.setPower(speed);
//                rightFront.setPower(speed);
//                rightBack.setPower(-speed);
//            }
//            else if (zAngle > 0) {
//                leftFront.setPower(-speed-zOrientation/100);
//                leftBack.setPower(speed+zOrientation/100);
//                rightFront.setPower(speed);
//                rightBack.setPower(-speed);
//            }
//            else if (zAngle < 0) {
//                leftFront.setPower(-speed);
//                leftBack.setPower(speed);
//                rightFront.setPower(speed+zOrientation/100);
//                rightBack.setPower(-speed-zOrientation/100);
//            }
//        }
//    }
//
//    public void gyro_rightstrafe(double speed){
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        // leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        // rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        // rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//
//        // double rotationsNeeded = inches/CIRCUMFERENCE;
//        // int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);
//
//        // init_IMU();
//
//        int zAngle = MR_Gyro.getIntegratedZValue();
//
//        // leftFront.setTargetPosition(encoderDrivingTarget);
//        // leftBack.setTargetPosition(-encoderDrivingTarget);
//        // rightFront.setTargetPosition(-encoderDrivingTarget);
//        // rightBack.setTargetPosition(encoderDrivingTarget);
//
//        leftFront.setPower(speed);
//        leftBack.setPower(-speed);
//        rightFront.setPower(-speed);
//        rightBack.setPower(speed);
//
//        // leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while(leftFront.isBusy() || rightFront.isBusy()){
//            if(zAngle == 0){
//                leftFront.setPower(speed);
//                leftBack.setPower(-speed);
//                rightFront.setPower(-speed);
//                rightBack.setPower(speed);
//            }
//            else if (zAngle > 0) {
//                leftFront.setPower(speed+zOrientation/100);
//                leftBack.setPower(-speed-zOrientation/100);
//                rightFront.setPower(-speed);
//                rightBack.setPower(speed);
//            }
//            else if (zAngle < 0) {
//                leftFront.setPower(speed);
//                leftBack.setPower(-speed);
//                rightFront.setPower(-speed-zOrientation/100);
//                rightBack.setPower(speed+zOrientation/100);
//            }
//        }
//    }

    public void Move_to_Position(double TargetPosition, double speed) {
        // Reset the encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put motors in encoder mode
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Turn on the motors using a moderate power
        leftFront.setPower(speed);
        rightBack.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        // Loop until the motor reaches its target
        while (leftFront.getCurrentPosition() < TargetPosition) {
            // nothing while the robot moves forward
        }
        // if(leftFront.getCurrentPosition() == TargetPosition){
        leftFront.setPower(ZERO);
        rightBack.setPower(ZERO);
        leftBack.setPower(ZERO);
        rightFront.setPower(ZERO);
        // }

        // Sleep a quarter second to let the robot stop
    }

    public void reset_encoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Move_to_Position_Negative(double TargetPosition, double speed) {
        // Reset the encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Put motors in encoder mode
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Turn on the motors using a moderate power
        leftFront.setPower(-speed);
        rightBack.setPower(-speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        // Loop until the motor reaches its target
        while (leftFront.getCurrentPosition() > TargetPosition) {
            // nothing while the robot moves forward
        }
        // if(leftFront.getCurrentPosition() == TargetPosition){
        leftFront.setPower(ZERO);
        rightBack.setPower(ZERO);
        leftBack.setPower(ZERO);
        rightFront.setPower(ZERO);


        // }

        // Sleep a quarter second to let the robot stop
    }

    public void zero(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }





    public void strafe_left(double power){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(-power);

    }

    public void strafe_right(double power){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(power);

    }

//    public void MR_gyroturn(double target) throws InterruptedException{
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        int zAccumulated = MR_Gyro.getIntegratedZValue();
//
//        double turnSpeed = 0.3;
//
//        while(Math.abs(zAccumulated - target) > 4){
//            if(zAccumulated > target) {
//
//                zAccumulated = MR_Gyro.getIntegratedZValue();
//                leftFront.setPower(turnSpeed);
//                leftBack.setPower(turnSpeed);
//                rightFront.setPower(-turnSpeed);
//                rightBack.setPower(-turnSpeed);
//            }
//
//            if(zAccumulated < target) {
//
//                zAccumulated = MR_Gyro.getIntegratedZValue();
//                leftFront.setPower(-turnSpeed);
//                leftBack.setPower(-turnSpeed);
//                rightFront.setPower(turnSpeed);
//                rightBack.setPower(turnSpeed);
//            }
//        }
//
//        zero();
//
//
//    }

    public void forward(double power){
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

    }

    public void backward(double power){
        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);

    }

    public void Move_Forward(double inches, double power){

        // leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        // leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        // rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        // rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = inches/CIRCUMFERENCE;
        int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);

        leftFront.setTargetPosition(encoderDrivingTarget);
        leftBack.setTargetPosition(encoderDrivingTarget);
        rightFront.setTargetPosition(encoderDrivingTarget);
        rightBack.setTargetPosition(encoderDrivingTarget);

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Move_Backward(double inches, double power){

        // leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        // leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        // rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        // rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = inches/CIRCUMFERENCE;
        int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);

        leftFront.setTargetPosition(-encoderDrivingTarget);
        leftBack.setTargetPosition(-encoderDrivingTarget);
        rightFront.setTargetPosition(-encoderDrivingTarget);
        rightBack.setTargetPosition(-encoderDrivingTarget);

        leftFront.setPower(-power);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        rightBack.setPower(-power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void turn_right(double power) {
        rightFront.setPower(power);
    }
    public void turn_left(double power) {
        leftFront.setPower(power);
    }

    public void move_motor(double speed) {
        leftFront.setPower(-speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(-speed);
    }

    public void gyro_leftstrafe(double speed) {


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        // leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        // rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        // rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0

        // double rotationsNeeded = inches/CIRCUMFERENCE;
        // int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);

        int zAngle = MR_Gyro.getIntegratedZValue();

        // leftFront.setTargetPosition(-encoderDrivingTarget);
        // leftBack.setTargetPosition(encoderDrivingTarget);
        // rightFront.setTargetPosition(encoderDrivingTarget);
        // rightBack.setTargetPosition(-encoderDrivingTarget);

        leftFront.setPower(-speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(-speed);

        // leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftFront.isBusy() || rightFront.isBusy()) {
            if (zAngle == 0) {
                leftFront.setPower(-speed);
                leftBack.setPower(speed);
                rightFront.setPower(speed);
                rightBack.setPower(-speed);
            } else if (zAngle > 0) {
                leftFront.setPower(-speed - zOrientation / 100);
                leftBack.setPower(speed + zOrientation / 100);
                rightFront.setPower(speed);
                rightBack.setPower(-speed);
            } else if (zAngle < 0) {
                leftFront.setPower(-speed);
                leftBack.setPower(speed);
                rightFront.setPower(speed + zOrientation / 100);
                rightBack.setPower(-speed - zOrientation / 100);
            }
        }
    }

    public void gyro_rightstrafe(double speed) {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        // leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        // rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        // rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0

        // double rotationsNeeded = inches/CIRCUMFERENCE;
        // int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);

        // init_IMU();

        int zAngle = MR_Gyro.getIntegratedZValue();

        // leftFront.setTargetPosition(encoderDrivingTarget);
        // leftBack.setTargetPosition(-encoderDrivingTarget);
        // rightFront.setTargetPosition(-encoderDrivingTarget);
        // rightBack.setTargetPosition(encoderDrivingTarget);

        leftFront.setPower(speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(speed);

        // leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftFront.isBusy() || rightFront.isBusy()) {
            if (zAngle == 0) {
                leftFront.setPower(speed);
                leftBack.setPower(-speed);
                rightFront.setPower(-speed);
                rightBack.setPower(speed);
            } else if (zAngle > 0) {
                leftFront.setPower(speed + zOrientation / 100);
                leftBack.setPower(-speed - zOrientation / 100);
                rightFront.setPower(-speed);
                rightBack.setPower(speed);
            } else if (zAngle < 0) {
                leftFront.setPower(speed);
                leftBack.setPower(-speed);
                rightFront.setPower(-speed - zOrientation / 100);
                rightBack.setPower(speed + zOrientation / 100);
            }
        }
    }
    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, angleUnit);
        return angles.firstAngle;
    }



    public void MR_gyrostraight(double inches, double speed) {

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double leftPower;
        double rightPower;
        int target = 0;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = inches / CIRCUMFERENCE;
        int encoderDrivingTarget = (int) (rotationsNeeded * ONEONEFIVEZERO_MOTOR_TICK_COUNT * 2);

        while (leftFront.getCurrentPosition() < encoderDrivingTarget || rightFront.getCurrentPosition() < encoderDrivingTarget) {
            int zAngle = MR_Gyro.getIntegratedZValue();

            leftPower = speed + (zAngle - target) / 100;
            rightPower = speed - (zAngle - target) / 100;

            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);

            leftFront.setTargetPosition(encoderDrivingTarget);
            leftBack.setTargetPosition(encoderDrivingTarget);
            rightFront.setTargetPosition(encoderDrivingTarget);
            rightBack.setTargetPosition(encoderDrivingTarget);

            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (leftFront.getCurrentPosition() > encoderDrivingTarget * 0.85) {
                leftFront.setPower(0.3);
                leftBack.setPower(0.3);
                rightFront.setPower(0.3);
                rightBack.setPower(0.3);

            }

        }
        zero();


    }

    public void MR_gyroreverse(double inches, double speed) {

        // leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double leftPower;
        double rightPower;
        int target = 0;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double rotationsNeeded = inches / CIRCUMFERENCE;
        int encoderDrivingTarget = (int) (rotationsNeeded * ONEONEFIVEZERO_MOTOR_TICK_COUNT * 2);

        while (leftFront.getCurrentPosition() > -encoderDrivingTarget || rightFront.getCurrentPosition() > -encoderDrivingTarget) {
            int zAngle = MR_Gyro.getIntegratedZValue();

            leftPower = -speed + (zAngle - target) / 100;
            rightPower = -speed - (zAngle - target) / 100;

            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);

            leftFront.setTargetPosition(-encoderDrivingTarget);
            leftBack.setTargetPosition(-encoderDrivingTarget);
            rightFront.setTargetPosition(-encoderDrivingTarget);
            rightBack.setTargetPosition(-encoderDrivingTarget);

            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        zero();
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void calibrate_MRgyro() {

        MR_Gyro.calibrate();

        while (MR_Gyro.isCalibrating()) {

        }
    }

//    public void MR_gyrostraight(double inches, double speed){
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        double leftPower;
//        double rightPower;
//        int target = 0;
//
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        double rotationsNeeded = inches/CIRCUMFERENCE;
//        int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);
//
//        while(leftFront.getCurrentPosition()<encoderDrivingTarget||rightFront.getCurrentPosition()<encoderDrivingTarget){
//            int zAngle = MR_Gyro.getIntegratedZValue();
//
//            leftPower = speed + (zAngle - target)/100;
//            rightPower = speed - (zAngle - target)/100;
//
//            leftPower = Range.clip(leftPower, -1, 1);
//            rightPower = Range.clip(rightPower, -1, 1);
//
//            leftFront.setTargetPosition(encoderDrivingTarget);
//            leftBack.setTargetPosition(encoderDrivingTarget);
//            rightFront.setTargetPosition(encoderDrivingTarget);
//            rightBack.setTargetPosition(encoderDrivingTarget);
//
//            leftFront.setPower(leftPower);
//            leftBack.setPower(leftPower);
//            rightFront.setPower(rightPower);
//            rightBack.setPower(rightPower);
//
//            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            if(leftFront.getCurrentPosition()>encoderDrivingTarget*0.85){
//                leftFront.setPower(0.3);
//                leftBack.setPower(0.3);
//                rightFront.setPower(0.3);
//                rightBack.setPower(0.3);
//
//            }
//
//        }
//        zero();
//
//
//    }
//
//    public void MR_gyroreverse(double inches, double speed){
//
//        // leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        double leftPower;
//        double rightPower;
//        int target = 0;
//
//        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
//        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        double rotationsNeeded = inches/CIRCUMFERENCE;
//        int encoderDrivingTarget = (int) (rotationsNeeded*ONEONEFIVEZERO_MOTOR_TICK_COUNT*2);
//
//        while(leftFront.getCurrentPosition()>-encoderDrivingTarget||rightFront.getCurrentPosition()>-encoderDrivingTarget){
//            int zAngle = MR_Gyro.getIntegratedZValue();
//
//            leftPower = -speed + (zAngle - target)/100;
//            rightPower = -speed - (zAngle - target)/100;
//
//            leftPower = Range.clip(leftPower, -1, 1);
//            rightPower = Range.clip(rightPower, -1, 1);
//
//            leftFront.setTargetPosition(-encoderDrivingTarget);
//            leftBack.setTargetPosition(-encoderDrivingTarget);
//            rightFront.setTargetPosition(-encoderDrivingTarget);
//            rightBack.setTargetPosition(-encoderDrivingTarget);
//
//            leftFront.setPower(leftPower);
//            leftBack.setPower(leftPower);
//            rightFront.setPower(rightPower);
//            rightBack.setPower(rightPower);
//
//            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        zero();
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//    }
//
//    public void calibrate_MRgyro(){
//
//        MR_Gyro.calibrate();
//
//        while (MR_Gyro.isCalibrating()){
//
//        }
//    }
//
//    public void edge_detection(){
//
//        leftFront.setPower(0.15);
//        leftBack.setPower(0.15);
//        rightFront.setPower(0.15);
//        rightBack.setPower(0.15);
//
//        while(color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)!=10){
//        }
//
//        zero();
//
//        while(rangeSide.rawUltrasonic() < 35){
//
//            strafe_left(0.35);
//        }
//
//        zero();
//    }

//    public void color_sense(int color){
//
//        leftFront.setPower(0.15);
//        leftBack.setPower(0.15);
//        rightFront.setPower(0.15);
//        rightBack.setPower(0.15);
//
//        while(color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)!=color){
//
//        }
//
//        zero();
//    }

//    public void color_sense_back(int color){
//
//        leftFront.setPower(-0.15);
//        leftBack.setPower(-0.15);
//        rightFront.setPower(-0.15);
//        rightBack.setPower(-0.15);
//
//        while(color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER)!=color){
//
//        }
//
//        zero();
//    }



    // public void strafe_right_encoder(int target, double power){
    //     leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
    //     leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
    //     rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets the counter of ticks to 0
    //     rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //     leftFront.setTargetPosition(-target);
    //     leftBack.setTargetPosition(target);
    //     rightFront.setTargetPosition(target);
    //     rightBack.setTargetPosition(-target);

    //     leftFront.setPower(-power);
    //     leftBack.setPower(power);
    //     rightFront.setPower(power);
    //     rightBack.setPower(-power);

    //     leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //     leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //     rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //     rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }

    // public void Init_IMU() {
    //     BNO055IMU.Parameters IMUparameters;

    //     // Create a new IMU parameters object
    //     IMUparameters = new BNO055IMU.Parameters();
    //     // Set the IMU mode to IMU so it automatically calibrates itself
    //     IMUparameters.mode = BNO055IMU.SensorMode.IMU;
    //     // Use degrees as our angle unit
    //     IMUparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    //     // Use meters per second as a unit of acceleration
    //     IMUparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    //     // Intialize IMU using these parameters
    //     imu.initialize(IMUparameters);
    // }

    // public void rotateCCW(double targetOrientationAngle) {
    //     float zOrientation;

    //     // Rotate in CCW direcction
    //     // Assumes we haven't turned more than 180 degrees
    //     // Get initial orientation about the Z axis
    //     zOrientation = getZAxisOrientation();
    //     // Set power so robot rotates in CCW direction
    //     leftFront.setPower(-POINT_TWO);
    //     rightBack.setPower(POINT_TWO);
    //     leftBack.setPower(-POINT_TWO);
    //     rightFront.setPower(POINT_TWO);
    //     // Loop until we've reached the target orientation
    //     while (zOrientation < targetOrientationAngle) {
    //       // Update current orientation about Z
    //       zOrientation = getZAxisOrientation();
    //     }
    //     // Stop the motors
    //     leftFront.setPower(ZERO);
    //     rightBack.setPower(ZERO);
    //     leftBack.setPower(ZERO);
    //     rightFront.setPower(ZERO);
    // }
}
