package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by riseo on 11/13/2016.
 */

@TeleOp(name="DemoBotDrive", group="OpMode")
//@Disabled

public class DemoBotDrive extends OpMode {

    float dl = -1;
    float dh = 1;

    float driveFactor = 1;

    //Motors

    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorCollect;

    private ElapsedTime runtime = new ElapsedTime();

    public DemoBotDrive() {
    }



    @Override
    public void init() {

        //initialization routine

        motorRight = hardwareMap.dcMotor.get("ma");
        motorLeft =  hardwareMap.dcMotor.get("mb");
        motorCollect = hardwareMap.dcMotor.get("mc");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorCollect.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorCollect.setPower(0);

        runtime.reset();

    }

    @Override
    public void loop() {

        telemetry.addData("LPower", motorLeft.getPower());
        telemetry.addData("RPower", motorRight.getPower());

        //telemetry for distance

        //variables for motor controls
        float rightpower = 2 * (gamepad1.right_stick_y) / driveFactor;
        float leftpower = 2 * (gamepad1.left_stick_y) / driveFactor;

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

        //Collection Commands
        if (gamepad1.a) {
            motorCollect.setPower (-1);//hi hailey and joe how are you guys doin????
       }
        if (gamepad1.b) {
            motorCollect.setPower (0);
        }
        if (gamepad1.y) {
            motorCollect.setPower (1);
        }

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
