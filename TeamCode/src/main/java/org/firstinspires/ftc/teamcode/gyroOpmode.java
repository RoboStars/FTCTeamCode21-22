package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class gyroOpmode extends OpMode {
    HardwareTestbot bot = new HardwareTestbot();
    double base_degrees;
    double last_time;
    @Override
    public void init() {
        bot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
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
            Thread.sleep(50);
        } catch (Exception e) { }
        while(!imu.isGyroCalibrated()) {
            telemetry.addData("Gyro Initializiation Status: ", imu.isGyroCalibrated());
            telemetry.update();
            try {
                Thread.sleep(50);
                Thread.yield();
            } catch (Exception e) {}
        }

        telemetry.addData("Gyro Initializiation Status: ", imu.isGyroCalibrated());
        telemetry.addData("Current Gyro Heading: ", bot.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        base_degrees = bot.getHeading(AngleUnit.DEGREES);
        last_time = getRuntime();
    }
    @Override
    public void loop() {
        telemetry.addData("Current Angle: ", bot.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        double imu_change;

        // calculate the shift/turn of the gyro
        //if (base_degrees > 0) {
        //    imu_change = bot.getHeading(AngleUnit.DEGREES) - base_degrees;
        //} else {
        //    imu_change = Math.abs(base_degrees - bot.getHeading(AngleUnit.DEGREES));
        // }

        if (getRuntime() - last_time < 0.1) {
            return;
        }

        last_time = getRuntime();
        if (bot.getHeading(AngleUnit.DEGREES) > -1 && bot.getHeading(AngleUnit.DEGREES) < 1 ) {
                bot.forward(0.1);
                telemetry.addData("Current State: ", "Straight");
                telemetry.update();
            } else {
                if (bot.getHeading(AngleUnit.DEGREES) < -1) {
                    bot.turn_right(0.5);
                    telemetry.addData("Current State: ", "Turning Left");
                    telemetry.addData("Current Angle: ", bot.getHeading(AngleUnit.DEGREES));
                    telemetry.update();

                }
                if (bot.getHeading(AngleUnit.DEGREES) > 1) {
                    bot.turn_left(0.5);
                    telemetry.addData("Current Angle: ", bot.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("Current State: ", "Turning Right");
                    telemetry.update();
                }
        }
    }
}
