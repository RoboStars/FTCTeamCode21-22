package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="TestBot: SimpleTeleopLeft", group="TestBot")
public class TestBotSimpleTeleopLeft extends OpMode {
    //HardwareTestbot robot = new HardwareTestbot();
    
    public DcMotor  rightFront = null;
    public DcMotor rightBack = null;
    
    
    
    double armPosition = 0.00;
    final double ARM_SPEED = 0.002;
    
    
    
 @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
       // robot.init(hardwareMap);
       
       
        rightFront = hardwareMap.dcMotor.get("frontL");
        rightBack = hardwareMap.dcMotor.get("backL");
        
        
        
        rightFront.setDirection(DcMotor.Direction.FORWARD); 
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        
       
       

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
         
        //  double Target = 0;
        //  robot.rightBack.setTargetPosition((int)Target);
        //  robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         rightFront.setPower(left);
         rightBack.setPower(right);
        // robot.rightDrive.setPower(right);

        // // Use gamepad left & right Bumpers to open and close the claw
        // if (gamepad1.right_bumper)
        //     clawOffset += CLAW_SPEED;
        // else if (gamepad1.left_bumper)
        //     clawOffset -= CLAW_SPEED;

        // // Move both servos to new position.  Assume servos are mirror image of each other.
        // clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        // robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        // robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        // if (gamepad1.y)
        //     armPosition += ARM_SPEED;
           
        // if (gamepad1.a)
        //     armPosition -= ARM_SPEED;
        
        // armPosition = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);    
     

        // // Send telemetry message to signify robot running;
        // telemetry.addData("arm",  "%.2f", armPosition);
        // telemetry.addData("left",  "%.2f", left);
        // telemetry.addData("right", "%.2f", right);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
 


