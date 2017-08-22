package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="CompDrive", group="OpMode")
//@Disabled

public class CompDrive extends OpMode {

    //Color Sensor Declarations
    I2cDevice colorA;
    I2cDevice colorB;
    I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;
    I2cDeviceSynch colorCreader;

    double flyPower = 0;

    double driveClipPlus = 1;
    double driveClipMinus = 1;

    float dl = -1;
    float dh = 1;

    double pos1 = 1;
    double pos2 = 0;
    double pos3 = 0.94;
    //double pos4 = 0.9;

    float driveFactor = 1;

    //Gyro Sensor Declaration
    GyroSensor gyroSensor;

    //Motors

    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorCollect;
    DcMotor motorConvey;
    DcMotor motorLeftFly;
    DcMotor motorRightFly;
    DcMotor motorSlide;

    CRServo LServo;
    //CRServo RServo;
    Servo BallServo;
    Servo SlideServo;
    Servo DeflectServo;
    //Servo BallSmacker;
    Servo FunnelL;
    Servo FunnelR;

    boolean RtBumpControl = false;
    boolean LtBumpControl = false;


    byte[] colorAcache;
    byte[] colorBcache;

    private ElapsedTime runtime = new ElapsedTime();

    public CompDrive() {
    }



    @Override
    public void init() {

        //initialization routine

        colorA = hardwareMap.i2cDevice.get("ca");
        colorB = hardwareMap.i2cDevice.get("cb");

        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x1e), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x20), false);

        colorAreader.engage();
        colorBreader.engage();

        gyroSensor = hardwareMap.gyroSensor.get("gyroSensor");

        motorRight = hardwareMap.dcMotor.get("ma");
        motorLeft =  hardwareMap.dcMotor.get("mb");
        motorCollect = hardwareMap.dcMotor.get("mc");
        motorConvey = hardwareMap.dcMotor.get("md");
        motorLeftFly =  hardwareMap.dcMotor.get("me");
        motorRightFly = hardwareMap.dcMotor.get("mf");
        motorSlide = hardwareMap.dcMotor.get("mg");

        LServo = hardwareMap.crservo.get("LServo");
       // RServo = hardwareMap.crservo.get("RServo");
        BallServo = hardwareMap.servo.get("BallServo");
        SlideServo = hardwareMap.servo.get("SlideServo");
        DeflectServo = hardwareMap.servo.get("DeflectServo");
        //BallSmacker = hardwareMap.servo.get("BallSmacker");
        FunnelL = hardwareMap.servo.get("FunnelL");
        FunnelR = hardwareMap.servo.get("FunnelR");

        //RServo.setDirection(CRServo.Direction.REVERSE);
        //BallServo.setDirection(Servo.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorCollect.setDirection(DcMotor.Direction.REVERSE);

        motorRightFly.setDirection(DcMotor.Direction.REVERSE);

        motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorCollect.setPower(0);
        motorConvey.setPower(0);
        motorLeftFly.setPower(0);
        motorRightFly.setPower(0);
        motorSlide.setPower(0);

        BallServo.setPosition(0.95);
        SlideServo.setPosition(0);
        DeflectServo.setPosition(0.93);

        //BallSmacker.setPosition(0.5);

        runtime.reset();

    }

    @Override
    public void start() {
        //resets the timer before start
    }

    @Override
    public void loop() {

        //Variables for color censors
        int iA = colorAcache[0];
        int iB = colorBcache[0];

        RtBumpControl = false;
        LtBumpControl = false;

        //Telemetry to display motor speeds
        telemetry.addData("RtBumpControl", RtBumpControl);
        telemetry.addData("LtBumpControl", LtBumpControl);
        telemetry.addData("LFly Power", motorLeftFly.getPower());
        telemetry.addData("RFly Power", motorRightFly.getPower());
        telemetry.addData("flyPower", flyPower);
        telemetry.addData("LPower", motorLeft.getPower());
        telemetry.addData("RPower", motorRight.getPower());

        telemetry.addData("Gyro", gyroSensor.getHeading());

        telemetry.addData("Servo Position", BallServo.getPosition());
        telemetry.addData("Deflector Position", DeflectServo.getPosition());

        //telemetry for distance


        //variables for motor controls
        float rightpower = 2 * (gamepad1.right_stick_y) / driveFactor;
        float leftpower = 2 * (gamepad1.left_stick_y) / driveFactor;
        float slidepower = gamepad2.right_stick_y;
        rightpower = Range.clip(rightpower, dl, dh);
        leftpower = Range.clip(leftpower, dl, dh);
        slidepower = Range.clip(slidepower, -1, 1);
        //rightpower = (float)scaleInput(rightpower);
        //leftpower =  (float)scaleInput(leftpower);
        //slidepower = (float)scaleInput(slidepower);
        motorRight.setPower(rightpower);
        motorLeft.setPower(leftpower);
        motorSlide.setPower(-slidepower);
        motorLeftFly.setPower(flyPower);
        motorRightFly.setPower(flyPower);

        if (gamepad1.dpad_down) {

            driveFactor = 3;
        }

        if (gamepad1.dpad_up) {

            driveFactor = 2;

        }


        //variables for servo controls


        if (gamepad1.right_bumper) {
            RtBumpControl = true;
            //RServo.setPower(-.5);

        }

        if (gamepad1.left_bumper) {
            LtBumpControl = true;
            LServo.setPower(.5);
        }


        if (RtBumpControl == false){

            float servpowR = gamepad1.right_trigger;
            servpowR = (float) scaleInput(servpowR);
            servpowR = Range.clip(servpowR, -1, 1);
            //RServo.setPower(servpowR);

        }

        if (LtBumpControl == false){

            float servpowL = -gamepad1.left_trigger;
            servpowL = Range.clip(servpowL, -1, 1);
            servpowL = (float)scaleInput(servpowL);
            LServo.setPower(servpowL);

        }


        //sets conveyor controls
        float conveyPower = gamepad2.left_stick_y;
        conveyPower = Range.clip(conveyPower, -1, 1);
        conveyPower = (float)scaleInput(conveyPower);
        motorConvey.setPower(conveyPower);

        //Collection Commands
        if (gamepad1.a) {
            motorCollect.setPower (-0.5);//hi  and joe how are you guys doin????
       }
        if (gamepad1.b) {
            motorCollect.setPower (0);
        }
        if (gamepad1.y) {
            motorCollect.setPower (0.5);
        }

        pos1 = Range.clip(pos1, 0, 1);
        pos2 = Range.clip(pos2, 0, 1);
        pos3 = Range.clip(pos3, 0, 1);
        //pos4 = Range.clip(pos4, 0, 1);

        //controls for servo on linear slide
        if(gamepad2.dpad_down){
            pos1 = .95;
        }
        if(gamepad2.dpad_up){
            pos1 = 0.05;
        }
        if(gamepad2.dpad_left){
            pos1 += .005;
            BallServo.setPosition(pos1);
        }
        BallServo.setPosition(pos1);

        SlideServo.setPosition(pos2);

        if(gamepad1.dpad_left){
            FunnelL.setPosition(0.7);
            FunnelR.setPosition(0.3);
        }

        if(gamepad1.dpad_right){
            FunnelL.setPosition(0.01);
            FunnelR.setPosition(0.99);
        }


        if(gamepad2.right_bumper) {
            pos2 += 0.02;
            SlideServo.setPosition(pos2);
        }
        if(gamepad2.left_bumper) {
            pos2 -= 0.02;
            SlideServo.setPosition(pos2);
        }

        DeflectServo.setPosition(pos3);

      /* if(gamepad1.dpad_left){
            pos3 += 0.0005;
            DeflectServo.setPosition(pos3);
        }

        if(gamepad1.dpad_right){
            pos3 -= 0.0005;
            DeflectServo.setPosition(pos3);
        } */


       //controls for flywheels
        if(gamepad2.y){
            flyPower = 0.58;
        }

        if(gamepad2.x){
            flyPower += 0.001;
        }

        if(gamepad2.b){
            flyPower -= 0.001;
        }

        if(gamepad2.a){
            flyPower = 0;
        }

        if(gamepad2.start){

            flyPower = -1;

        }

        /* // comment this out if not working
        if(gamepad1.dpad_up){
            dl = -1;
            dh = 1;
        }

        if(gamepad1.dpad_down){
            dl = -1/2;
            dh = 1/2;
        } */

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
