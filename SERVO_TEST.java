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

@TeleOp(name="ServoTest", group="OpMode")
//@Disabled

public class SERVO_TEST extends OpMode {

    double pos1 = 0;

    Servo Servo;
    CRServo CRServo;

    private ElapsedTime runtime = new ElapsedTime();

    public SERVO_TEST() {
    }



    @Override
    public void init() {

        Servo = hardwareMap.servo.get("Servo");
        CRServo = hardwareMap.crservo.get("CRServo");

        Servo.setPosition(0);
        CRServo.setPower(0);

        runtime.reset();

    }

    @Override
    public void loop() {

        telemetry.addData("Servo Position", Servo.getPosition());

        pos1 = Range.clip(pos1, 0, 1);

        //controls for servo on linear slide
        if(gamepad1.dpad_down){
            pos1 = 0;
        }
        if(gamepad1.dpad_up){
            pos1 = 1;
        }

        if(gamepad1.y){
            CRServo.setPower(0.8);
        }

        if(gamepad1.b){
            CRServo.setPower(0);
        }

        if(gamepad1.a){
            CRServo.setPower(-0.8);
        }

        Servo.setPosition(pos1);

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
