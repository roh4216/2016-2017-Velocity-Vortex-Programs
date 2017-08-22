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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Type, name, group
@Autonomous(name = "ShootCapBLUE", group = "IterativeOpMode")
@Disabled
//OpMode details

public class $ShootAndCapBLUE extends OpMode {

    //Declare Gyro and Range sensor
    GyroSensor gyroSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;

    //Variable for range sensor
    double rangedist;

    //Variable for Gyro Sensor
    int GyroPos;

    //Declare variables for servo

    final static double DISTANCE = 207872;


    //Declare servo
    CRServo LServo;
    CRServo RServo;
    Servo BallServo;
    Servo SlideServo;

    //Declare moyopt
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


    //Overrides and runsOpmode
    @Override
    public void init() {
        //Telemetry for robot status
        telemetry.addData("Status", "Initialized");



        //Range and Gyro sensor hardware mapping
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        gyroSensor = hardwareMap.gyroSensor.get("gyroSensor");

        //Servo hardware mapping
        LServo = hardwareMap.crservo.get("LServo");
        RServo = hardwareMap.crservo.get("RServo");
        BallServo = hardwareMap.servo.get("BallServo");
        SlideServo = hardwareMap.servo.get("SlideServo");

        BallServo.setPosition(0.95);
        SlideServo.setPosition(0);


        //Declares the names of the color sensors in config
        colorA = hardwareMap.i2cDevice.get("ca");
        colorB = hardwareMap.i2cDevice.get("cb");

        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x1e), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x30), false);

        //Engages color sensors???
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

        //Get distance from range sensor
        rangedist = rangeSensor.getDistance(DistanceUnit.CM);

        //telemetry for range sensor
        telemetry.addData("raw ultrasonic ", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical ", rangeSensor.rawOptical());
        telemetry.addData("cm optical ", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm ", "%.2f cm", rangedist);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Update telemetry
        telemetry.update();

        //Calibarte gyro sensor
        gyroSensor.calibrate();

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
        rangedist = rangeSensor.getDistance(DistanceUnit.CM);

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


        // Telemetry for the Range Sensor (Includes both Ultrasonic values and Optical values
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangedist);

        //telemetry to display timer
        telemetry.addData("Elapsed Time", runtime);

        //telemetry to display Gyro position
        telemetry.addData("06 Gyro Heading", gyroSensor.getHeading());

        telemetry.addData("Motor Target", DISTANCE);
        telemetry.addData("Left Motor Position", motorLeft.getCurrentPosition());
        telemetry.addData("Right Motor Position", motorRight.getCurrentPosition());



        // Updates the telemetry
        telemetry.update();

        switch (case_switch) {

            case 0:

                motorLeftFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorRightFly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                motorLeftFly.setPower(.23);
                motorRightFly.setPower(-.23);

                motorCollect.setPower(0.5);

                motorLeft.setPower(.2);
                motorRight.setPower(.2);

                if (motorLeft.getCurrentPosition() > 3300) {

                    runtime.reset();
                    motorLeft.setPower(0);
                    motorRight.setPower(0);
                    case_switch = 1;
                }

                break;

            case 1:

                //launches particle
                motorConvey.setPower(.4);


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

                motorLeft.setPower(.225);
                motorRight.setPower(.2);

                if (motorLeft.getCurrentPosition() > 1600) {

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


