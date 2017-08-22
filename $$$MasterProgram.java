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

import com.qualcomm.hardware.adafruit.BNO055IMU;
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
import com.vuforia.ar.pl.SystemTools;

//Type, name, group
@Autonomous(name = "Master Program", group = "IterativeOpMode")
@Disabled
//OpMode details

public class $$$MasterProgram extends OpMode {

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
    CRServo RServo;
    Servo BallServo;
    Servo SlideServo;
    Servo DeflectServo;
    Servo FunnelServo;

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
    int case_switch;
    int init_switch = 0;
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

        //Servo hardware mapping
        LServo = hardwareMap.crservo.get("LServo");
        RServo = hardwareMap.crservo.get("RServo");
        BallServo = hardwareMap.servo.get("BallServo");
        SlideServo = hardwareMap.servo.get("SlideServo");
        DeflectServo = hardwareMap.servo.get("DeflectServo");
        FunnelServo = hardwareMap.servo.get("FunnelServo");


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
        DeflectServo.setPosition(0.93);
        FunnelServo.setPosition(0);



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

        switch(init_switch){

            case 0:

                telemetry.addLine("MAIN MENU");
                telemetry.addLine();
                telemetry.addLine("> A - Red Side");
                telemetry.addLine("> B - Blue Side");
                case_switch = 999;

                if(gamepad1.a){
                    telemetry.clearAll();
                    init_switch = 1;
                }

                if(gamepad1.b){
                    telemetry.clearAll();
                    init_switch = 2;
                }

                break;

            case 1:

                telemetry.addLine("You selected RED SIDE.");
                telemetry.addLine();
                telemetry.addLine("> X - Shoot, beacons, park");
                telemetry.addLine("> Y - Shoot, center");
                telemetry.addLine("Press START to go back to main menu.");

                if(gamepad1.x){
                    telemetry.clearAll();
                    init_switch = 3;
                }

                if(gamepad1.y){
                    telemetry.clearAll();
                    init_switch = 4;
                }


                if(gamepad1.start){
                    telemetry.clearAll();
                    init_switch = 0;

                }

                break;

            case 2:
                telemetry.addLine("You selected BLUE SIDE.");
                telemetry.addLine();
                telemetry.addLine("> X - Shoot, beacons, park");
                telemetry.addLine("> Y - Shoot, center");
                telemetry.addLine("Press START to go back to main menu.");

                if(gamepad1.x){
                    telemetry.clearAll();
                    init_switch = 5;
                }

                if(gamepad1.y){
                    telemetry.clearAll();
                    init_switch = 6;
                }

                if(gamepad1.start){
                    telemetry.clearAll();
                    init_switch = 0;

                }
                break;

            case 3:
                telemetry.addLine("You selected RED - SHOOT, BEACONS, PARK.");
                telemetry.addLine();
                telemetry.addLine("> A - CENTER park");
                telemetry.addLine("> B - RAMP park");
                telemetry.addLine("Press START to go back to main menu.");

                if(gamepad1.a){
                    telemetry.clearAll();
                    init_switch = 7;
                }

                if(gamepad1.b){
                    telemetry.clearAll();
                    init_switch = 8;
                }


                if(gamepad1.start){
                    telemetry.clearAll();
                    init_switch = 0;

                }

                break;

            case 4:
                telemetry.addLine("You selected RED - SHOOT, CENTER.");
                telemetry.addLine();
                telemetry.addLine("> A - NO wait");
                telemetry.addLine("> B - 10 SEC wait");
                telemetry.addLine("Press START to go back to main menu.");

                if(gamepad1.a){
                    telemetry.clearAll();
                    init_switch = 9;
                }

                if(gamepad1.b){
                    telemetry.clearAll();
                    init_switch = 10;
                }


                if(gamepad1.start){
                    telemetry.clearAll();
                    init_switch = 0;

                }

                break;

            case 5:
                telemetry.addLine("You selected BLUE - SHOOT, BEACONS, PARK.");
                telemetry.addLine();
                telemetry.addLine("> A - CENTER park");
                telemetry.addLine("> B - RAMP park");
                telemetry.addLine("Press START to go back to main menu.");

                if(gamepad1.a){
                    telemetry.clearAll();
                    init_switch = 11;
                }

                if(gamepad1.b){
                    telemetry.clearAll();
                    init_switch = 12;
                }


                if(gamepad1.start){
                    telemetry.clearAll();
                    init_switch = 0;

                }

                break;

            case 6:
                telemetry.addLine("You selected BLUE - SHOOT, CENTER.");
                telemetry.addLine();
                telemetry.addLine("> A - NO wait");
                telemetry.addLine("> B - 10 SEC wait");
                telemetry.addLine("Press START to go back to main menu.");

                if(gamepad1.a){
                    telemetry.clearAll();
                    init_switch = 13;
                }

                if(gamepad1.b){
                    telemetry.clearAll();
                    init_switch = 14;
                }


                if(gamepad1.start){
                    telemetry.clearAll();
                    init_switch = 0;

                }

                break;

            case 7:
                telemetry.addLine("You selected RED - SHOOT, BEACONS, CENTER PARK.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 0;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 8:
                telemetry.addLine("You selected RED - SHOOT, BEACONS, RAMP PARK.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 100;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 9:

                telemetry.addLine("You selected RED - SHOOT, CENTER, NO WAIT");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 200;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 10:
                telemetry.addLine("You selected RED - SHOOT, CENTER, 10 SEC WAIT.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 300;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 11:
                telemetry.addLine("You selected BLUE - SHOOT, BEACONS, CENTER PARK.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 400;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 12:
                telemetry.addLine("You selected BLUE - SHOOT, BEACONS, RAMP PARK.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 500;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 13:
                telemetry.addLine("You selected BLUE - SHOOT, CENTER, NO WAIT.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 600;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;

            case 14:
                telemetry.addLine("You selected BLUE - SHOOT, CENTER, 10 SEC WAIT.");
                telemetry.addLine();
                telemetry.addLine("Press START to go back to main menu.");
                case_switch = 700;

                if(gamepad1.start){
                    telemetry.clearAll();
                    case_switch = 0;
                    init_switch = 0;

                }

                break;
        }

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

            //Shoot, beacons, park center - RED

            case 0:

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.66);
                motorRightFly.setPower(-.66);

                if(runtime.seconds() > 1){
                    motorConvey.setPower(.7);
                    motorCollect.setPower(0.5);
                }

                if (runtime.seconds() > 3.5){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorConvey.setPower(0);
                    motorCollect.setPower(0);
                    motorLeftFly.setPower(0);
                    motorRightFly.setPower(0);
                    runtime.reset();
                    case_switch = 1;
                }

                break;


            case 1:
                //JUST TURN 35

                // Set targetheading variable to what we want it to turn to
                targetheading = -35;

                //Drive power depending on what the gyro is at
                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 2);
                    rightPower = (0);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (0);
                    rightPower = (-driveSteering * 2);
                }
                else {

                    //Switch to case 3
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 2;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 2;
                }

                break;

            case 2:

                //JUST GO STRAIGHT

                // Set variables
                ForwardDrivePower = 0.6;
                targetheading = -35;


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
                    runtime.reset();

                    case_switch = 3;
                }

                break;

            case 3:
                //TURN BACK to Zero
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveSteering * 1.2);
                    rightPower = (-driveSteering * 1.2);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveSteering * 1.2);
                    rightPower = (-driveSteering * 1.2);
                }

                else {
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 4;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){

                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 4;

                }

                break;


            case 4:

                //GOES STRAIGHT FORWARD TO HIT THE FAR BEACON

                if (runtime.seconds() < 1.5){
                    ForwardDrivePower = 0.17;
                }
                if (runtime.seconds() > 1.5) {
                    ForwardDrivePower = 0.14;
                }
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower) * 1.1;
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
                    case_switch = 5;

                }

                break;

            case 5:
                //HIT THE FIRST BEACON!!!
                LServo.setPower(-1);

                if (runtime.seconds() > 3){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 6;
                }


                break;

            case 6:
                //GO REVERSE TO NEXT BEACON

                LServo.setPower(0.9);


                if (runtime.seconds() < 1.5){

                    ForwardDrivePower = -0.35;
                }

                if (runtime.seconds() > 1.5){

                    ForwardDrivePower = -0.18;
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
                    case_switch = 7;

                }

                break;

            case 7:

                //HIT SECOND BEACON

                LServo.setPower(-1);

                if (runtime.seconds() > 3){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 8;
                }

                break;

            case 8:

                //DRIVES IN CURVE TO POSITION TO SHOOT THE BALLS

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                LServo.setPower(.9);

                targetheading = 98;

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

                    case_switch = 9;
                }

                break;


            case 9:

                //TURN OFF FLYWHEELS AND CONVEYOR, GOES FORWARD TO CENTER

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                motorLeft.setPower(.6);
                motorRight.setPower(.6);

                if (motorRight.getCurrentPosition() > 800) {

                    //AFTER ~9in, SWITCHES CASE TO STOP
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 10;
                }

                break;

            case 10:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                break;


            case 100:

                //Shoot, beacons, park ramp - RED

                targetheading = 10;

                //Drive power depending on what the gyro is at
                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 2.5);
                    rightPower = (0);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (0);
                    rightPower = (-driveSteering * 2.5);
                }

                else if (runtime.seconds() > 2){
                    //Switch to case 1
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 101;
                }

                else {

                    //Switch to case 1
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 101;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.66);
                motorRightFly.setPower(-.66);

                break;

            case 101:

                motorConvey.setPower(.7);
                motorCollect.setPower(0.5);

                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 2.5);
                    rightPower = (0);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (0);
                    rightPower = (-driveSteering * 2.5);
                }

                else {

                    //Switch to case 1
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                }

                if (runtime.seconds() > 2.5){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorConvey.setPower(0);
                    motorCollect.setPower(0);
                    motorLeftFly.setPower(0);
                    motorRightFly.setPower(0);
                    runtime.reset();
                    case_switch = 103;
                }

                break;


            case 102:
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
                    case_switch = 103;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 103;
                }

                break;

            case 103:

                //JUST GO STRAIGHT

                // Set variables
                ForwardDrivePower = 0.4;
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
                    runtime.reset();
                    case_switch = 105;
                }

                break;

            case 104:
                //TURN BACK to Zero
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveSteering * 1.2);
                    rightPower = (-driveSteering * 1.2);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveSteering * 1.2);
                    rightPower = (-driveSteering * 1.2);
                }
                else {
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 105;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 105;
                }

                break;


            case 105:

                //GOES STRAIGHT FORWARD TO HIT THE FAR BEACON

                if (runtime.seconds() < 1.5){
                    ForwardDrivePower = 0.1;
                }
                if (runtime.seconds() > 1.5) {
                    ForwardDrivePower = 0.12;
                }
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveStraight + ForwardDrivePower) * 0.4;
                    rightPower = (-driveStraight + ForwardDrivePower);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveStraight + ForwardDrivePower);
                    rightPower = (-driveStraight + ForwardDrivePower) * 1.15;
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
                    case_switch = 106;

                }

                break;

            case 106:
                //HIT THE FIRST BEACON!!!
                LServo.setPower(-1);

                if (runtime.seconds() > 3 ){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 107;
                }


                break;

            case 107:
                //GO REVERSE TO NEXT BEACON

                LServo.setPower(0.9);


                if (runtime.seconds() < 1.5){

                    ForwardDrivePower = -0.27;
                }

                if (runtime.seconds() > 1.5){

                    ForwardDrivePower = -0.12;
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
                if (runtime.seconds() > 2 && iA > 9 && iA < 12){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 108;

                }

                break;

            case 108:

                //HIT SECOND BEACON

                LServo.setPower(-1);

                if (runtime.seconds() > 3 ){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 109;
                }

                break;

            case 109:

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                LServo.setPower(0.9);

                motorLeft.setPower(0.8);
                motorRight.setPower(0.8);

                if(motorLeft.getCurrentPosition() > 600){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 110;
                }

                break;

            case 110:

                motorLeft.setPower(-.6);
                motorRight.setPower(-.2);

                if(runtime.seconds() > 0.7){

                    runtime.reset();
                    case_switch = 111;
                }

                break;

            case 111:

                motorLeft.setPower(-.1);
                motorRight.setPower(-.6);

                break;



            case 200:

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.63);
                motorRightFly.setPower(-.63);

                motorLeft.setPower(.2);
                motorRight.setPower(.2);

                if (motorLeft.getCurrentPosition() > 2200) {

                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 201;
                }

                break;

            case 201:

                //launches particle
                motorConvey.setPower(.5);


                if (runtime.seconds() > 5){
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 202;
                }

                break;


            case 202:


                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                motorLeft.setPower(.2);
                motorRight.setPower(.225);

                if (motorLeft.getCurrentPosition() > 2200) {

                    case_switch = 203;
                }

                break;


            case 203:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                break;



            case 300:

                motorLeft.setPower(0);
                motorRight.setPower(0);
                runtime.reset();
                case_switch = 301;

                break;

            case 301:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                if (runtime.seconds() > 7){

                    case_switch = 302;
                }

                break;

            case 302:

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.63);
                motorRightFly.setPower(-.63);

                motorLeft.setPower(.2);
                motorRight.setPower(.2);

                if (motorLeft.getCurrentPosition() > 2200) {

                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 303;
                }

                break;

            case 303:

                //launches particle
                motorConvey.setPower(.5);


                if (runtime.seconds() > 5){
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 304;
                }

                break;


            case 304:


                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                motorLeft.setPower(.2);
                motorRight.setPower(.225);

                if (motorLeft.getCurrentPosition() > 2200) {

                    case_switch = 305;
                }

                break;


            case 305:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                break;



            case 400:

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.66);
                motorRightFly.setPower(-.66);

                if(runtime.seconds() > 1){
                    motorConvey.setPower(.7);
                    motorCollect.setPower(0.5);
                }

                if (runtime.seconds() > 3.5){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorConvey.setPower(0);
                    motorCollect.setPower(0);
                    motorLeftFly.setPower(0);
                    motorRightFly.setPower(0);
                    runtime.reset();
                    case_switch = 401;
                }

                break;


            case 401:
                //JUST TURN 35

                // Set targetheading variable to what we want it to turn to
                targetheading = 35;

                //Drive power depending on what the gyro is at
                if (GyroPos < targetheading - 2) {
                    leftPower = (driveSteering * 2);
                    rightPower = (0);
                }
                else if (GyroPos > targetheading + 2) {
                    leftPower = (0);
                    rightPower = (-driveSteering * 2);
                }
                else {

                    //Switch to case 3
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 402;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 402;
                }

                break;

            case 402:

                //JUST GO STRAIGHT

                // Set variables
                ForwardDrivePower = 0.6;
                targetheading = 35;


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

                if (u2 < 17 && u2 > 0){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 403;
                }

                break;

            case 403:
                //TURN BACK to Zero
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveSteering * 1.2);
                    rightPower = (-driveSteering * 1.2);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveSteering * 1.2);
                    rightPower = (-driveSteering * 1.2);
                }
                else {
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 404;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5) {

                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 404;

                }

                break;


            case 404:

                //GOES STRAIGHT FORWARD TO HIT THE FAR BEACON

                if (runtime.seconds() < 1.5){
                    ForwardDrivePower = 0.21;
                }
                if (runtime.seconds() > 1.5) {
                    ForwardDrivePower = 0.18;
                }
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveStraight + ForwardDrivePower) * 1.1;
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
                if (iB > 2 && iB < 5){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 405;

                }

                break;

            case 405:
                //HIT THE FIRST BEACON!!!
                RServo.setPower(-1);

                if (runtime.seconds() > 3){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 406;
                }


                break;

            case 406:
                //GO REVERSE TO NEXT BEACON

                RServo.setPower(0.9);


                if (runtime.seconds() < 1.5){

                    ForwardDrivePower = -0.35;
                }

                if (runtime.seconds() > 1.5){

                    ForwardDrivePower = -0.18;
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
                if (runtime.seconds() > 2 && iB > 2 && iB < 5){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 407;

                }

                break;

            case 407:

                //HIT SECOND BEACON

                RServo.setPower(-1);

                if (runtime.seconds() > 3){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 408;
                }

                break;


            case 408:

                //DRIVES IN CURVE TO POSITION TO SHOOT THE BALLS

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                RServo.setPower(.9);

                targetheading = -105;

                if (GyroPos > targetheading){

                    motorLeft.setPower(.3);
                    motorRight.setPower(0.95);

                }
                else {

                    motorLeft.setPower(leftPower);
                    motorRight.setPower(rightPower);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    runtime.reset();

                    case_switch = 409;
                }

                break;


            case 409:

                //TURN OFF FLYWHEELS AND CONVEYOR, GOES FORWARD TO CENTER

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                motorLeft.setPower(.6);
                motorRight.setPower(.6);

                if (motorRight.getCurrentPosition() > 800) {

                    //AFTER ~9in, SWITCHES CASE TO STOP
                    runtime.reset();
                    case_switch = 410;
                }

                break;


            case 410:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                break;




            case 500:

                targetheading = -10;

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
                    case_switch = 501;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.66);
                motorRightFly.setPower(-.66);

                break;

            case 501:

                motorConvey.setPower(.7);
                motorCollect.setPower(0.5);

                if (runtime.seconds() > 2.5){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorConvey.setPower(0);
                    motorCollect.setPower(0);
                    motorLeftFly.setPower(0);
                    motorRightFly.setPower(0);
                    runtime.reset();
                    case_switch = 502;
                }

                break;


            case 502:
                //JUST TURN 27

                // Set targetheading variable to what we want it to turn to
                targetheading = 27;

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
                    case_switch = 503;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 503;
                }

                break;

            case 503:

                //JUST GO STRAIGHT

                // Set variables
                ForwardDrivePower = 0.6;
                targetheading = 27;


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

                if (u2 < 17 && u2 > 0){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 504;
                }

                break;

            case 504:
                //TURN BACK to Zero
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveSteering * 1.1);
                    rightPower = (-driveSteering * 1.1);
                }
                else if (GyroPos > targetheading + 1) {
                    leftPower = (driveSteering * 1.1);
                    rightPower = (-driveSteering * 1.1);
                }
                else {
                    leftPower = (0);
                    rightPower = (0);
                    runtime.reset();
                    case_switch = 505;
                }

                motorLeft.setPower(leftPower);
                motorRight.setPower(rightPower);

                if (runtime.seconds() > 2.5){
                    leftPower = (0);
                    rightPower = (0);
                    case_switch = 505;
                }

                break;


            case 505:

                //GOES STRAIGHT FORWARD TO HIT THE FAR BEACON

                if (runtime.seconds() < 1.5){
                    ForwardDrivePower = 0.21;
                }
                if (runtime.seconds() > 1.5) {
                    ForwardDrivePower = 0.18;
                }
                targetheading = 0;

                if (GyroPos < targetheading - 1) {
                    leftPower = (driveStraight + ForwardDrivePower)  * 1.1;
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
                if (iB > 2 && iB < 5){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 506;

                }

                break;

            case 506:
                //HIT THE FIRST BEACON!!!
                RServo.setPower(-1);

                if (runtime.seconds() > 2.5 ){
                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 507;
                }


                break;

            case 507:
                //GO REVERSE TO NEXT BEACON

                RServo.setPower(0.9);


                if (runtime.seconds() < 1.5){

                    ForwardDrivePower = -0.35;
                }

                if (runtime.seconds() > 1.5){

                    ForwardDrivePower = -0.18;
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
                if (runtime.seconds() > 3 && iB > 2 && iB < 5){

                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 508;

                }

                break;

            case 508:

                //HIT SECOND BEACON

                RServo.setPower(-1);

                if (runtime.seconds() > 2.5 ){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    runtime.reset();
                    case_switch = 509;
                }

                break;

            case 509:

                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                RServo.setPower(0.9);

                motorLeft.setPower(0.8);
                motorRight.setPower(0.8);

                if(motorLeft.getCurrentPosition() > 600){
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    runtime.reset();
                    case_switch = 510;
                }

                break;

            case 510:

                motorLeft.setPower(-.2);
                motorRight.setPower(-.6);

                if(runtime.seconds() > 0.7){

                    runtime.reset();
                    case_switch = 511;
                }

                break;

            case 511:

                motorLeft.setPower(-.6);
                motorRight.setPower(-.1);

                break;

            case 999:

                break;

        }


        }


    @Override
    public void stop() {
    }
}


