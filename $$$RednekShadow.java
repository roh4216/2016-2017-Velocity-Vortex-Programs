/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */



/* Auto_1

 */
//package
package org.firstinspires.ftc.teamcode;

//import

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//Type, name, group
@Autonomous(name = "RednekShadow", group = "IterativeOpMode")
//This is the program that shadows them and stops to block defense
//@Disabled
//OpMode details

public class $$$RednekShadow extends OpMode {

    //Declare Gyro and Range sensor
    GyroSensor gyroSensor;
    UltrasonicSensor range1;
    UltrasonicSensor range2;

    //Variable for range sensor
    double rangedist;
    double distance;

    //Variable for Gyro Sensor
    int GyroPos;

    double driveSteering;
    double driveStraight;

    //Variables for gyro proportional steering

    double Proportionalpower = 0.0;
    double ForwardDrivePower = 0.0;
    double rightPower = 0.0;
    double leftPower = 0.0;
    int targetheading = -35;

    //Declare servos
    CRServo LServo;
   // CRServo RServo;
    Servo BallServo;
    Servo SlideServo;
    Servo DeflectServo;
    Servo FunnelL;
    Servo FunnelR;

    //Declare motors
    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorCollect;
    DcMotor motorConvey;
    DcMotor motorLeftFly;
    DcMotor motorRightFly;

    //Declare stuff for color sensors
    byte[] colorAcache;
    byte[] colorBcache;
    byte[] colorCcache;

    //Declare color sensors
    I2cDevice colorA;
    I2cDevice colorB;
    I2cDevice colorC;

    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;
    I2cDeviceSynch colorCreader;

    //Case variables
    int case_switch = 0;
    int case_pos = 0;
    int pos = 0;
    int loc = 0;

    int ball = 0;

    //Declare elapsed time
    private ElapsedTime runtime = new ElapsedTime();


    //Overrides and runs Opmode
    @Override
    public void init() {
        //Telemetry for robot status
        telemetry.addData("Status", "Initialized");

        //Range and Gyro sensor hardware mapping
        range1 = hardwareMap.ultrasonicSensor.get("u1");
        range2 = hardwareMap.ultrasonicSensor.get("u2");
        gyroSensor = hardwareMap.gyroSensor.get("gyroSensor");

        //Servo hardware mapping
        LServo = hardwareMap.crservo.get("LServo");
       // RServo = hardwareMap.crservo.get("RServo");
        BallServo = hardwareMap.servo.get("BallServo");
        SlideServo = hardwareMap.servo.get("SlideServo");
        DeflectServo = hardwareMap.servo.get("DeflectServo");
        FunnelL = hardwareMap.servo.get("FunnelL");
        FunnelR = hardwareMap.servo.get("FunnelR");


        //Declares the names of the color sensors in config
        colorA = hardwareMap.i2cDevice.get("ca");
        colorB = hardwareMap.i2cDevice.get("cb");
        colorC = hardwareMap.i2cDevice.get("cc");

        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x1e), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x30), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x20), false);

        //Engages color sensors
        colorAreader.engage();
        colorBreader.engage();
        colorCreader.engage();

        /* Set mode for the color sensors.
        * The first number in the parentheses, 3, sets the color sensors into "Command" mode.
        * The second number in the parentheses, 1, turns the LED's on the color sensors off (Passive Mode).
        * Inserting a 0 instead of a 1 turns the LED on (Active Mode).
        */
        colorAreader.write8(3, 1);
        colorBreader.write8(3, 1);
        colorCreader.write8(3, 0);

        //Declares names of motors in config
        motorRight = hardwareMap.dcMotor.get("ma");
        motorLeft = hardwareMap.dcMotor.get("mb");
        motorCollect = hardwareMap.dcMotor.get("mc");
        motorConvey = hardwareMap.dcMotor.get("md");
        motorLeftFly = hardwareMap.dcMotor.get("me");
        motorRightFly = hardwareMap.dcMotor.get("mf");

        //Sets the direction of the motors
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorConvey.setDirection(DcMotor.Direction.REVERSE);


        //sets encoders
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Update telemetry
        telemetry.update();

        //Set initialization position for ball and slide servos
        BallServo.setPosition(1);
        SlideServo.setPosition(0);
        DeflectServo.setPosition(0.94);

        // Calibrate the Gyro Sensor and add telemetry for it
        gyroSensor.calibrate();
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        //telemetry.update();

// While the gyro is calibrating wait
        while (gyroSensor.isCalibrating()) {
            int i=1;
        }

// Tell us when the gyro is done calibrating
        telemetry.addData(">", "Gyro Calibrated.");

        // Reset runtime
        runtime.reset();
        telemetry.clearAll();

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        //resets the timer before start
        gyroSensor.resetZAxisIntegrator();
        runtime.reset();
    }

    @Override
    public void loop() {
        //Set the target heading. (What we want the gyro to go to at this point)
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
        double baseTurnPowerFactor = 0.003;
        //MIN Power to have when turning so the robot does keep moving - this gets determined by trial and error
        double baseTurnPowerMin = 0.;

        //THE calculation to determine how much power for the turn
        //driveStraight gives the power without baseTurn variable

        //*****************DRIVE CALCS**************************************

        if (headingError > 0) {
            driveSteering = ((double) headingError * baseTurnPowerFactor) + baseTurnPowerMin;
            driveStraight = ((double) headingError * baseTurnPowerFactor);
        }
        else {
            driveSteering = ((double) headingError * baseTurnPowerFactor) - baseTurnPowerMin;
            driveStraight = ((double) headingError * baseTurnPowerFactor);
        }


        // Clips the range of the steering and going straight
        driveSteering = Range.clip(driveSteering, -0.2, 0.2);
        driveStraight = Range.clip(driveStraight, -0.2, 0.2);

        //Use this number different;y and clip it for maximum power
        //Proportionalpower = driveSteering;
        //Proportionalpower = Range.clip(Proportionalpower, dl, dh);


        //IF GYRO IS NOT WITHIN PLUS/MINUS 1 degree add proportional power to turn it, if its not just go straight

        double u1 = range1.getUltrasonicLevel();
        double u2 = range2.getUltrasonicLevel();

        // Read color sensors
        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);

        // Sets a variable for the color caches
        int iA = colorAcache[0];
        int iB = colorBcache[0];
        int iC = colorCcache[0];

        //Telemetry to display program's position
        telemetry.addData("Case: ", case_switch);

        telemetry.addData("Actual Gyro Heading", GyroPos);
        telemetry.addData("Target Heading", targetheading);
        telemetry.addData("Heading Error", headingError);
        telemetry.addData("drive Steering Power", driveSteering);
        telemetry.addData("proportional Power", Proportionalpower);

        //Telemetry for range sensors
        telemetry.addData("u1", u1);
        telemetry.addData("u2", u2);

        // Telemetry for the color sensors
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData(" integer color", iA);

        telemetry.addData("2 #B", colorBcache[0] & 0xFF);
        telemetry.addData("String", 1);

        telemetry.addData("3 #C", colorCcache[0] & 0xFF);
        telemetry.addData(" integer color", iC);

        //telemetry to display speed
        telemetry.addData("LSpeed", motorLeft.getPower());
        telemetry.addData("RSpeed", motorRight.getPower());

        //telemetry to display timer
        telemetry.addData("Elapsed Time", runtime);

        //Telemetry for motor encoder counts
        telemetry.addData("Left Motor Position", motorLeft.getCurrentPosition());
        telemetry.addData("Right Motor Position", motorRight.getCurrentPosition());

        // Updates the telemetry
        telemetry.update();

        if (iC > 1 && iC < 5) {
            motorCollect.setPower(-.5);
            motorConvey.setPower(-0.25);
        }

        if (iC > 9 && iC < 12) {
            motorConvey.setPower(0.95);
            ball = 1;
        }

        if (iC == 0){
            motorConvey.setPower(0);
        }



        //Start cases
        switch (case_switch) {

            case 0:

                //turn 45

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                driveSteering = Range.clip(driveSteering, -0.1, 0.1);
                driveStraight = Range.clip(driveStraight, -0.1, 0.1);

                targetheading = -30;

                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 0.3);
                    rightPower = (-driveSteering * 0.4);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (driveSteering * 0.65);
                    rightPower = (-driveSteering * 0.9);
                }
                else {
                    leftPower = 0;
                    rightPower = 0;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if(runtime.seconds() > 3){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 1;
                }

               break;

            case 1:

                //drive 25

                motorCollect.setPower(-0.5);

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                targetheading = -30;

                ForwardDrivePower = 0.6;

                if (GyroPos < targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else {
                    leftPower = ForwardDrivePower;
                    rightPower = ForwardDrivePower;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if(motorLeft.getCurrentPosition() > 3000){
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 2;
                }

                break;

            case 2:

                //turn 0

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                targetheading = -5;

                if (GyroPos < targetheading - 4) {
                    leftPower = (driveSteering);
                    rightPower = (-driveSteering * 0.9);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (driveSteering * 0.6);
                    rightPower = (-driveSteering * 0.85);
                }
                else {
                    leftPower = 0;
                    rightPower = 0;
                }

                if(runtime.seconds() > 4){
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 3;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                break;

            case 3:

                motorCollect.setPower(-0.5);

                //drive 0 and stop

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                targetheading = 0;

                ForwardDrivePower = 0.6;

                if (GyroPos < targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else {
                    leftPower = ForwardDrivePower;
                    rightPower = ForwardDrivePower;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if(motorLeft.getCurrentPosition() > 800){
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 4;
                }

                break;

            case 4:

                motorCollect.setPower(0);

                motorLeft.setPower(0);
                motorRight.setPower(0);

                if(runtime.seconds() > 11){
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 5;
                }

                break;

            case 5:

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                driveSteering = Range.clip(driveSteering, -0.1, 0.1);
                driveStraight = Range.clip(driveStraight, -0.1, 0.1);

                targetheading = -30;

                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 0.3);
                    rightPower = (-driveSteering * 0.4);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (driveSteering * 0.65);
                    rightPower = (-driveSteering * 1.1);
                }
                else {
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 6;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if(runtime.seconds() > 6){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 6;
                }

                break;

            case 6:

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                targetheading = -30;

                ForwardDrivePower = -0.6;

                if (GyroPos < targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else {
                    leftPower = ForwardDrivePower;
                    rightPower = ForwardDrivePower;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if(motorLeft.getCurrentPosition() < -2400){
                    leftPower = 0;
                    rightPower = 0;
                    runtime.reset();
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 7;
                }

                break;

            case 7:

                leftPower = 0;
                rightPower = 0;

                break;
        }


        }


    @Override
    public void stop() {
    }
}


