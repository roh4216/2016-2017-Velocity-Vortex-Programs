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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//Type, name, group
@Autonomous(name = "BeaconCapShootFirstRED", group = "IterativeOpMode")
@Disabled
//OpMode details

public class BeaconCapShootFirstRED extends OpMode {

    //Declare Gyro and Range sensor
    GyroSensor gyroSensor;
    UltrasonicSensor range1;
    UltrasonicSensor range2;
    OpticalDistanceSensor ods;

    double light;

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
    CRServo RServo;
    Servo BallServo;
    Servo SlideServo;
    Servo DeflectServo;

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

    //Declare color sensors
    I2cDevice colorA;
    I2cDevice colorB;

    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;

    //Case variables
    int case_switch = 0;
    int case_pos = 0;

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
        ods = hardwareMap.opticalDistanceSensor.get("ods");

        //Servo hardware mapping
        LServo = hardwareMap.crservo.get("LServo");
        RServo = hardwareMap.crservo.get("RServo");
        BallServo = hardwareMap.servo.get("BallServo");
        SlideServo = hardwareMap.servo.get("SlideServo");
        DeflectServo = hardwareMap.servo.get("DeflectServo");


        //Declares the names of the color sensors in config
        colorA = hardwareMap.i2cDevice.get("ca");
        colorB = hardwareMap.i2cDevice.get("cb");

        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x1e), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x30), false);

        //Engages color sensors
        colorAreader.engage();
        colorBreader.engage();

        /* Set mode for the color sensors.
        * The first number in the parentheses, 3, sets the color sensors into "Command" mode.
        * The second number in the parentheses, 1, turns the LED's on the color sensors off (Passive Mode).
        * Inserting a 0 instead of a 1 turns the LED on (Active Mode).
        */
        colorAreader.write8(3, 1);
        colorBreader.write8(3, 1);

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
        DeflectServo.setPosition(0.95);

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

        double FWDDrivePower = 0.0;

        // Reset runtime
        runtime.reset();


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
        double baseTurnPowerFactor = 0.004;
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
        driveSteering = Range.clip(driveSteering, -0.15, 0.15);
        driveStraight = Range.clip(driveStraight, -0.15, 0.15);

        //Use this number different;y and clip it for maximum power
        //Proportionalpower = driveSteering;
        //Proportionalpower = Range.clip(Proportionalpower, dl, dh);


        //IF GYRO IS NOT WITHIN PLUS/MINUS 1 degree add proportional power to turn it, if its not just go straight

        double u1 = range1.getUltrasonicLevel();
        double u2 = range2.getUltrasonicLevel();

        light = ods.getLightDetected();

        // Read color sensors
        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);

        // Sets a variable for the color caches
        int iA = colorAcache[0];
        int iB = colorBcache[0];

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

        telemetry.addData("ODS", light);

        // Telemetry for the color sensors
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData(" integer color", iA);

        telemetry.addData("2 #B", colorBcache[0] & 0xFF);
        telemetry.addData("String", 1);

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

        //Start cases
        switch (case_switch) {

            case 0:

                targetheading = 10;

                //Drive power depending on what the gyro is at
                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 3);
                    rightPower = (0);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (0);
                    rightPower = (-driveSteering * 3);
                }
                else {

                    //Switch to case 1
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 1;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.7);
                motorRightFly.setPower(-.7);

                break;

            case 1:

                motorConvey.setPower(.7);
                motorCollect.setPower(0.5);

                if (runtime.seconds() > 2.5){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorConvey.setPower(0);
                    motorCollect.setPower(0);
                    motorLeftFly.setPower(0);
                    motorRightFly.setPower(0);
                    case_switch = 2;
                }

                break;


            case 2:
            //JUST TURN 27

                // Set targetheading variable to what we want it to turn to
                targetheading = -27;

                //Drive power depending on what the gyro is at
                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering);
                    rightPower = (-driveSteering);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (driveSteering);
                    rightPower = (-driveSteering);
                }
                else {

                    //Switch to case 1
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 3;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                break;

            case 3:

                //JUST GO STRAIGHT

              // Set variables
              ForwardDrivePower = 0.6;
              targetheading = -27;


                if (GyroPos < targetheading - 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else {
                    leftPower = ForwardDrivePower;
                    rightPower = ForwardDrivePower;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                //when it gets to under 17cm, stops and switches case

                if (u1 < 17 && u1 > 0){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);

                    case_switch = 4;
                }

                break;

            case 4:
            //TURN BACK to Zero
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveSteering);
                    rightPower = (-driveSteering);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveSteering);
                    rightPower = (-driveSteering);
                }
                else {
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 5;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                break;


            case 5:

                //GOES STRAIGHT FORWARD TO HIT THE FAR BEACON

                if (runtime.seconds() < 1.5){
                    ForwardDrivePower = 0.27;
                }
                if (runtime.seconds() > 1.5) {
                    ForwardDrivePower = 0.21;
                }
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else {
                    leftPower = ForwardDrivePower;
                    rightPower = ForwardDrivePower;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                //What to do when it sees red
                if (iA > 9 && iA < 12){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 6;

                }

                break;

            case 6:
            //HIT THE FIRST BEACON!!!
                LServo.setPower(-.6);

                if (runtime.seconds() > 2.5 ){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 7;
                }


                break;

            case 7:
                //GO REVERSE TO NEXT BEACON

                LServo.setPower(0.9);


                if (runtime.seconds() < 2){

                    ForwardDrivePower = -0.35;
                }

                if (runtime.seconds() > 2){

                    ForwardDrivePower = -0.22;
                }

                targetheading = 0;

                if (GyroPos < targetheading) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else {
                    leftPower = ForwardDrivePower;
                    rightPower = ForwardDrivePower;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);


                //When it sees red:
               if (runtime.seconds() > 3 && iA > 9 && iA < 12){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 8;

                }

                break;

            case 8:

                //HIT SECOND BEACON

                LServo.setPower(-.6);

                if (runtime.seconds() > 2.5 ){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 10;
                }

                break;

            case 9:

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                if (light > 0.28){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 10;
                }

                else if (motorLeft.getCurrentPosition() > 1100){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 10;

                }

                LServo.setPower(.9);
                motorLeft.setPower(0.22);
                motorRight.setPower(0.22);

                break;

            case 10:

                //DRIVES IN CURVE TO POSITION TO SHOOT THE BALLS

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                LServo.setPower(.9);

                targetheading = 105;

                if (GyroPos < targetheading){

                    motorLeft.setPower(.95);
                    motorRight.setPower(0.3);

                }
                else {

                    motorLeft.setPower(leftPower);
                    motorRight.setPower(rightPower);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    runtime.reset();

                    case_switch = 11;
                }

                break;


            case 11:

                //TURN OFF FLYWHEELS AND CONVEYOR, GOES FORWARD TO CENTER

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                motorLeft.setPower(.8);
                motorRight.setPower(.8);

                if (motorRight.getCurrentPosition() > 1500) {

                    //AFTER ~9in, SWITCHES CASE TO STOP

                    case_switch = 12;
                }

                break;

            case 12:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                break;

            case 99:

                //Stops everything

                LServo.setPower(0);
                RServo.setPower(0);
                motorLeft.setPower(0);
                motorRight.setPower(0);
                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                break;
        }


        }


    @Override
    public void stop() {
    }
}


