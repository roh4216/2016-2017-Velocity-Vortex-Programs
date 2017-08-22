package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by riseo on 11/13/2016.
 */

@TeleOp(name="TestBotDrive", group="OpMode")
//@Disabled

public class TestBotDrive extends OpMode {

    //Color Sensor Declarations



    GyroSensor gyro;
    AnalogInput touch;

    double driveClipPlus = 1;
    double driveClipMinus = 1;

    float dl = -1;
    float dh = 1;

    float driveFactor = 1;

    //Motors

    DcMotor motorLeft;
    DcMotor motorRight;

    private ElapsedTime runtime = new ElapsedTime();

    public TestBotDrive() {
    }

    @Override
    public void init() {

        //initialization routine

        gyro = hardwareMap.gyroSensor.get("gyro");

        touch = hardwareMap.analogInput.get("touch");

        motorRight = hardwareMap.dcMotor.get("ma");
        motorLeft =  hardwareMap.dcMotor.get("mb");

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setPower(0);
        motorRight.setPower(0);

        runtime.reset();


    }

    @Override
    public void loop() {

        //Variables for color censors

        //Telemetry to display motor speeds
        telemetry.addData("LPower", motorLeft.getPower());
        telemetry.addData("RPower", motorRight.getPower());

        telemetry.addData("Gyro Pos", gyro.getHeading());


        //telemetry for distance


        //variables for motor controls
        float rightpower = 2 * (gamepad1.right_stick_y) / driveFactor;
        float leftpower = 2 * (gamepad1.left_stick_y) / driveFactor;
        float slidepower = gamepad2.right_stick_y;
        rightpower = Range.clip(rightpower, dl, dh);
        leftpower = Range.clip(leftpower, dl, dh);
        motorRight.setPower(rightpower);
        motorLeft.setPower(leftpower);

        if (gamepad1.dpad_down) {

            driveFactor = 3;
        }

        if (gamepad1.dpad_up) {

            driveFactor = 2;

        }


        //variables for servo controls


    }

    @Override
    public void stop() {

    }

    //values to scale power
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}