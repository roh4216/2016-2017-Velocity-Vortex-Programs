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

//Type, name, group
@Autonomous(name = "WAITShootCapBOTHSIDES", group = "IterativeOpMode")
@Disabled
//OpMode details

public class $ShootAndCapWAIT extends OpMode {

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
    int case_switch = 99;
    int case_pos = 0;

    //Declare elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    //Overrides and runsOpmode
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
        DeflectServo.setPosition(0.96);


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        //resets the timer before start
        runtime.reset();
    }

    @Override
    public void loop() {
        //Gets Gyro position
        GyroPos = gyroSensor.getHeading();
        //Gets range sensor distance
        // Read color sensors
        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);


        // Sets a variable for the color caches
        int iA = colorAcache[0];
        int iB = colorBcache[0];

        //Telemetry to display program's position
        telemetry.addData("Case: ", case_switch);

        // Telemetry for the color sensors
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData(" integer color", iA);

        telemetry.addData("2 #B", colorBcache[0] & 0xFF);
        telemetry.addData("String", 1);

        //telemetry to display timer
        telemetry.addData("Elapsed Time", runtime);

        //telemetry to display Gyro position
        telemetry.addData("06 Gyro Heading", gyroSensor.getHeading());

        telemetry.addData("Left Motor Position", motorLeft.getCurrentPosition());
        telemetry.addData("Right Motor Position", motorRight.getCurrentPosition());



        // Updates the telemetry
        telemetry.update();

        switch (case_switch) {

            case 99:

                motorLeft.setPower(0);
                motorRight.setPower(0);
                runtime.reset();
                case_switch = 98;

                break;

            case 98:

                motorLeft.setPower(0);
                motorRight.setPower(0);

                if (runtime.seconds() > 7){

                    case_switch = 0;
            }

                break;

            case 0:

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
                    case_switch = 1;
                }

                break;

            case 1:

                //launches particle
                motorConvey.setPower(.5);


                if (runtime.seconds() > 5){
                    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    case_switch = 2;
                }

                break;


            case 2:


                motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorConvey.setPower(0);
                motorLeftFly.setPower(0);
                motorRightFly.setPower(0);
                motorCollect.setPower(0);

                motorLeft.setPower(.2);
                motorRight.setPower(.225);

                if (motorLeft.getCurrentPosition() > 2200) {

                    case_switch = 3;
                }

                break;


            case 3:

                motorLeft.setPower(0);
                motorRight.setPower(0);

        }

    }
    @Override
    public void stop() {
    }
}


