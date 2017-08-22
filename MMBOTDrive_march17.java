package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by riseo on 11/13/2016.
 */

@TeleOp(name="MMBOTDriveNEW", group="MMBOT")
//@Disabled

public class MMBOTDrive_march17 extends OpMode {

    //Color Sensor Declarations

    byte[] colorAcache;
    byte[] colorBcache;

    I2cDevice colorA;
    I2cDevice colorB;

    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;

    int GyroPos;

    int iA;
    int iB;

    //MOTORS
    DcMotor motorLeft;
    DcMotor motorRight;

    //GYRO STARTING TARGET
    int targetheading = -45;

    //SENSORS
    UltrasonicSensor Ultra1;
    UltrasonicSensor Ultra2;
    GyroSensor gyroSensor;



    private ElapsedTime runtime = new ElapsedTime();

    public MMBOTDrive_march17() {
    }



    @Override
    public void init() {

        //initialization routine

        //MOTORS
        motorRight = hardwareMap.dcMotor.get("m1");
        motorLeft =  hardwareMap.dcMotor.get("m2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);


        //SENSORS
        //COLOR SENSORS for when using multiple
        colorA = hardwareMap.i2cDevice.get("c1");
        colorB = hardwareMap.i2cDevice.get("c2");


        //NOTE:  The Address refers to 8bithex 0x70 = 7bithex 0x38 =
        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x1e), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x20), false);

        colorAreader.engage();
        colorBreader.engage();

        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);

        gyroSensor = hardwareMap.gyroSensor.get("g1");

        Ultra1 = hardwareMap.ultrasonicSensor.get("u1");
        Ultra2 = hardwareMap.ultrasonicSensor.get("u2");

        gyroSensor.calibrate();


// Calibrate the gyro

        gyroSensor.calibrate();
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        //telemetry.update();

// While the gyro is calibrating wait
        while (gyroSensor.isCalibrating()) {
                    int i=1;
        }

// Tell us when the gyro is done calibrating
        telemetry.addData(">", "Gyro Calibrated.");

        double FWDDrivePower = 0.0;

        runtime.reset();


    }

    @Override
    public void loop() {

        float rightPower = gamepad1.right_stick_y;
        float leftPower = gamepad1.left_stick_y;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

        //Variables for color censors
        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);
        iA = colorAcache[0];
        iB = colorBcache[0];

        telemetry.addData("LPower", motorLeft.getPower());
        telemetry.addData("RPower", motorRight.getPower());
        telemetry.addData("Gyro", gyroSensor.getHeading());

        telemetry.addData("DistanceUltra1", Ultra1);
        telemetry.addData("DistanceUltra2", Ultra2);
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData(" integer color", iA);
        telemetry.addData("2 #B", colorBcache[0] & 0xFF);
        telemetry.addData(" integer color", iB);
        telemetry.addData("Actual Gyro Heading", GyroPos);

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
