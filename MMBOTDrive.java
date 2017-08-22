package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by riseo on 11/13/2016.
 */

@Autonomous(name="MMBOTDrive", group="MMBOT")
@Disabled

public class MMBOTDrive extends OpMode {

    //Color Sensor Declarations

    byte[] colorAcache;
    byte[] colorBcache;

    I2cDevice colorA;
    I2cDevice colorB;

    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;

    //VARIABLES
    //double flyPower = 0;
    double driveClipPlus = 1;
    double driveClipMinus = 1;
    double driveSteering;

    double dl = -.15;
    double dh = .15;
    double pos1 = 0;

    double Proportionalpower = 0.0;
    double ForwardDrivePower = 0.0;
    double rightPower = 0.0;
    double leftPower = 0.0;

    int GyroPos;
    //int GyroPos2;

    int iA;
    int iB;

    int case_switch = 0;

    boolean TouchCondition;

    //Gyro Sensor Declaration
    //GyroSensor gyroSensor;
    //ModernRoboticsI2cRangeSensor rangeSensor;


    //MOTORS
    DcMotor motorLeft;
    DcMotor motorRight;

    //GYRO STARTING TARGET
    int targetheading = -45;

    //SENSORS
    UltrasonicSensor Ultra1;
    UltrasonicSensor Ultra2;
    //CompassSensor Cmpass;
    TouchSensor touchSensor;
    GyroSensor gyroSensor;



    private ElapsedTime runtime = new ElapsedTime();

    public MMBOTDrive() {
    }



    @Override
    public void init() {

        //initialization routine

        //MOTORS
        motorRight = hardwareMap.dcMotor.get("m1");
        motorLeft =  hardwareMap.dcMotor.get("m2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);


        //SENSORS
        //rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        //COLOR SENSORS for when using multiple
        colorA = hardwareMap.i2cDevice.get("c1");
        colorB = hardwareMap.i2cDevice.get("c2");


        //NOTE:  The Address refers to 8bithex 0x70 = 7bithex 0x38 =
        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x38), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x20), false);

        colorAreader.engage();
        colorBreader.engage();

        colorAreader.write8(3, 1);
        colorBreader.write8(3, 1);

        gyroSensor = hardwareMap.gyroSensor.get("g1");

        Ultra1 = hardwareMap.ultrasonicSensor.get("u1");
        Ultra2 = hardwareMap.ultrasonicSensor.get("u2");
        //Cmpass - hardwareMap.compassSensor.get("cp1");
        touchSensor = hardwareMap.touchSensor.get("t1");

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

        //FORWARD POWER
        if(gamepad1.y){
            ForwardDrivePower = ForwardDrivePower + .01;
        }

        if(gamepad1.a){
            ForwardDrivePower = ForwardDrivePower - .01;
        }




        //ADJUST TARGETHEADING FROM INITIAL VALUE
        if(gamepad1.b){
            targetheading = targetheading + 1;
        }

        if(gamepad1.x){
            targetheading = targetheading - 1;
        }
        targetheading = Range.clip(targetheading, -179, 179);


        //GYRO POSITIONING READING
        GyroPos = gyroSensor.getHeading();
        //THIS SETS UP THE GYRO TO BE -180 to 180 versus 0 to 360
        if (GyroPos > 180)
            {
                        GyroPos = GyroPos -360;
            }

        //CALCULATE ERROR BETWEEN HEADING and Targetheading
        int headingError = targetheading - GyroPos;

        //FACTOR to multiply to heading error times (ie. if if the gyro is off by 2 degrees, the power is equal to 2 x baseTurnPowerFactor
        //CONSIDER THE CLIP WHEN YOU DO THIS
        double baseTurnPowerFactor = 0.005;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.055;

        //THE calculation to determine how much power for the turn


        if (headingError > 0) {
            driveSteering = ((double) headingError * baseTurnPowerFactor) + baseTurnPowerMin;
        }
        else {
            driveSteering = ((double) headingError * baseTurnPowerFactor) - baseTurnPowerMin;
        }


        //Use this number different;y and clip it for maximum power
        Proportionalpower = driveSteering;
        Proportionalpower = Range.clip(Proportionalpower, dl, dh);


        //IF GYRO IS NOT WITHIN PLUS/MINUS 1 degree add proportional power to turn it, if its not just go straight
        if (GyroPos < targetheading - 1) {
            leftPower = (Proportionalpower + ForwardDrivePower);
            rightPower = (-Proportionalpower + ForwardDrivePower);
        }
        else if (GyroPos > targetheading + 1) {
            leftPower = (Proportionalpower + ForwardDrivePower);
            rightPower = (-Proportionalpower + ForwardDrivePower);
        }
        else {
            leftPower = ForwardDrivePower;
            rightPower = ForwardDrivePower;
        }

        telemetry.addData("right power", rightPower);
        telemetry.addData("left power", leftPower);
        //SET MOTOR POWER AFTER ALL OF THE CALCULATIONS - KEEPS IT CLEAN
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);


        TouchCondition = touchSensor.isPressed();

        //Variables for color censors
        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);
        iA = colorAcache[0];
        iB = colorBcache[0];

        //telemetry.addData("LPower", motorLeft.getPower());
        //telemetry.addData("RPower", motorRight.getPower());

        //telemetry.addData("DistanceUltra1", Ultra1);
        //telemetry.addData("DistanceUltra2", Ultra2);
        //telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        //telemetry.addData(" integer color", iA);
        //telemetry.addData("2 #B", colorBcache[0] & 0xFF);
        //telemetry.addData(" integer color", iB);
        telemetry.addData("Actual Gyro Heading", GyroPos);
        telemetry.addData("Target Heading", targetheading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("drive Steering Power", driveSteering);
        telemetry.addData("proportional Power", Proportionalpower);


        //NOT USING CURRENTLY
        /*
        switch (case_switch){

            case 0:

                if (runtime.seconds() > 4){
                    case_switch = 1;
                }

                break;

            case 1:

                motorLeft.setPower(leftpower);
                motorRight.setPower(rightpower);

                break;


        }
        */
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
