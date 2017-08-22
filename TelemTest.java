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
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Type, name, group
@Autonomous(name = "TelemTest", group = "IterativeOpMode")
//@Disabled
//OpMode details

public class TelemTest extends OpMode {

    GyroSensor gyroSensor;

    UltrasonicSensor range1;
    UltrasonicSensor range2;

    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor motorCollect;
    DcMotor motorConvey;
    DcMotor motorLeftFly;
    DcMotor motorRightFly;

    byte[] colorAcache;
    byte[] colorBcache;
    byte[] colorCcache;


    I2cDevice colorA;
    I2cDevice colorB;
    I2cDevice colorC;

    I2cDeviceSynch colorAreader;
    I2cDeviceSynch colorBreader;
    I2cDeviceSynch colorCreader;


    double rangedist;

    int case_switch = 99;
    int case_pos = 0;

    //double time = getRuntime();

    //int GyroStart;
    int GyroPos;


    //declare elapsed time
    private ElapsedTime runtime = new ElapsedTime();


    //Overrides and runsOpmode
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        gyroSensor = hardwareMap.gyroSensor.get("gyroSensor");

        range1 = hardwareMap.ultrasonicSensor.get("u1");
        range2 = hardwareMap.ultrasonicSensor.get("u2");

        //Declares the names of the color sensors in config
        colorA = hardwareMap.i2cDevice.get("ca");
        colorB = hardwareMap.i2cDevice.get("cb");
        colorC = hardwareMap.i2cDevice.get("cc");

        colorAreader = new I2cDeviceSynchImpl(colorA, new I2cAddr(0x1e), false);
        colorBreader = new I2cDeviceSynchImpl(colorB, new I2cAddr(0x30), false);
        colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x20), false);


        //Engages color sensors???
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
        //colorDreader.write8(3, 0);
        //colorFreader.write8(3, 0);


        //Declares names of motors in config
        motorRight = hardwareMap.dcMotor.get("ma");
        motorLeft = hardwareMap.dcMotor.get("mb");
        motorCollect = hardwareMap.dcMotor.get("mc");
        motorConvey = hardwareMap.dcMotor.get("md");
        motorLeftFly = hardwareMap.dcMotor.get("me");
        motorRightFly = hardwareMap.dcMotor.get("mf");

        motorRight.setDirection(DcMotor.Direction.REVERSE);
        //motorConvey.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        gyroSensor.calibrate();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

        runtime.reset();
    }

    @Override
    public void loop() {

        double u1 = range1.getUltrasonicLevel();
        double u2 = range2.getUltrasonicLevel();

        GyroPos = gyroSensor.getHeading();

        // Read color sensors
        colorAcache = colorAreader.read(0x04, 1);
        colorBcache = colorBreader.read(0x04, 1);
        colorCcache = colorCreader.read(0x04, 1);
        //colorDcache = colorDreader.read(0x04, 1);
        //colorFcache = colorFreader.read(0x04, 1);


        // Sets a variable for the color caches
        int iA = colorAcache[0];
        int iB = colorBcache[0];
        int iC = colorCcache[0];
        //int iD = colorDcache[0];
        //int iF = colorFcache[0];


        // Telemetry for the color sensors
        telemetry.addData("1 #A", colorAcache[0] & 0xFF);
        telemetry.addData(" integer color", iA);

        telemetry.addData("2 #B", colorBcache[0] & 0xFF);
        telemetry.addData("String", iB);

        telemetry.addData("3 #C", colorCcache[0] & 0xFF);
        telemetry.addData(" integer color", iC);


        //telemetry.addData("3 #C", colorCcache[0] & 0xFF);
        //telemetry.addData(" integer color", iC);

        //telemetry.addData("4 #D", colorDcache[0] & 0xFF);
        //telemetry.addData(" integer color", iD);

        //telemetry.addData("5 #F", colorFcache[0] & 0xFF);
        //telemetry.addData(" integer color", iF);



        // Telemetry for the Range Sensor (Includes both Ultrasonic values and Optical values

        telemetry.addData("u1", u1);
        telemetry.addData("u2", u2);

        telemetry.addData("LPos", motorLeft.getCurrentPosition());
        telemetry.addData("RPos", motorRight.getCurrentPosition());

        telemetry.addData("Elapsed Time", runtime);

        telemetry.addData("06 Gyro Heading", gyroSensor.getHeading());

        //double time = getRuntime();
        //runtime.reset();
        // Updates the telemetry

        telemetry.update();

        switch (case_switch) {

            case 99:

                motorLeft.setPower(0.2);
                motorRight.setPower(0.2);

                //stop();


        }

    }
    @Override
    public void stop() {
    }
}


