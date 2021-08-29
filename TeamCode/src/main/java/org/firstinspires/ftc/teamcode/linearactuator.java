package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/*
Logic for the encoder incremental movement for Sat.


 */


@TeleOp(name="Ball intake", group="TestBot")
@Disabled
public class linearactuator extends OpMode {
    testbotdjaoiwejfae robot = new testbotdjaoiwejfae();

    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.linearactuator1.setPower(0);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    private void WaitAndPower(int miliseconds){

        double currentTime = getRuntime();
        double waitUntil = currentTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
            robot.linearactuator1.setPower(0.1);
        }

    }
    @Override
    public void loop() {
        if(gamepad1.a) {
            WaitAndPower(5000);

        }

    }

    @Override
    public void stop() {
    }



}
