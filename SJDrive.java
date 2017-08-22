package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by riseo on 11/13/2016.
 */

@TeleOp(name="SJDrive", group="OpMode")
//@Disabled

public class SJDrive extends OpMode {

    float dl = -1;
    float dh = 1;

    float driveFactor = 1;

    double speed = 0;

    //Motors

    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorIntake;
    DcMotor motorUseless;

    boolean motorintake = false;

    private ElapsedTime runtime = new ElapsedTime();

    public SJDrive() {
    }

    @Override
    public void init() {

        //initialization routine



        motorRight = hardwareMap.dcMotor.get("ma");
        motorLeft =  hardwareMap.dcMotor.get("mb");
        motorIntake = hardwareMap.dcMotor.get("mi");
        motorUseless = hardwareMap.dcMotor.get("mu");


        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorIntake.setPower(0);
        motorUseless.setPower(0);

    }

    @Override
    public void loop() {

        //Variables for color censors

        //Telemetry to display motor speeds
        telemetry.addData("LPower", motorLeft.getPower());
        telemetry.addData("RPower", motorRight.getPower());

        //variables for motor controls
        float rightpower = 2 * (gamepad1.right_stick_y);
        float leftpower = 2 * (gamepad1.left_stick_y);
        rightpower = Range.clip(rightpower, dl, dh);
        leftpower = Range.clip(leftpower, dl, dh);
        motorRight.setPower(rightpower);
        motorLeft.setPower(leftpower);

        //variables for intake
        if (gamepad1.right_bumper) {
            motorintake = true;
            motorIntake.setPower(-.9);

        }

        if (gamepad1.left_bumper) {
            motorintake = true;
            motorIntake.setPower(.9);
        }


        if (gamepad1.left_bumper&&gamepad1.right_bumper){
            motorintake = false;
            motorIntake.setPower(0);

        }

        if (gamepad1.y){
           speed += 0.1;
        }

        if (gamepad1.a){
            speed = 0;
        }

        if (gamepad1.b){
            speed -= 0.1;
        }

        motorUseless.setPower(speed);




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