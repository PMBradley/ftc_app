/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OffSeason2019.Hardware;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class TankDriveExample {
    //Declaration of variables and objects
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_Drive = null;
    private DcMotor right_Drive = null;
    private BNO055IMU gyro = null;

    private long move_End_Time = 0;// declared globaly to the class so the variables stay even when the move functions stop being called
    private double target_Heading = 0;
    private double original_Drive_Power = 0;
    private double left_Drive_Power = 0;
    private double right_Drive_Power = 0;
    private double turn_Move_Power = 0;


    TankDriveExample(DcMotor left_Motor, DcMotor right_Motor, BNO055IMU hub_Gyro){ // constructor
        left_Drive = left_Motor;
        right_Drive = right_Motor;
        gyro = hub_Gyro;
    }


    private double getGyroHeading(){ // put into a separate method to reduce clutter
        //Formats Gyro reading into Degrees
        double rawHeading = -AngleUnit.DEGREES.fromUnit(gyro.getAngularOrientation().angleUnit, gyro.getAngularOrientation().firstAngle);//only works after gyro is initialized

        return rawHeading;
    }


    public void startMove(double drive_Power, double turn_Power, int run_miliseonds, double turn_degrees){
        move_End_Time = (long)runtime.milliseconds() + run_miliseonds;

        target_Heading = getGyroHeading() + turn_degrees; // takes the current position and add

        turn_Move_Power = turn_Power;
        original_Drive_Power = drive_Power;


        left_Drive_Power = drive_Power + turn_Move_Power;
        right_Drive_Power = drive_Power - turn_Move_Power;

        left_Drive.setPower(left_Drive_Power);
        right_Drive.setPower(right_Drive_Power);
    }


    public boolean continueMove() { // checks to see if motors should keep moving, and if not, stop the motors
        long current_Time = (long) runtime.milliseconds();
        double current_Heading = getGyroHeading();
        boolean stillDriving = true;

        if (current_Time < move_End_Time) {
            stillDriving = true;
            if (current_Heading > target_Heading && turn_Move_Power > 0) { // if greater than target and moving away
                turn_Move_Power = turn_Move_Power * -1; // reverse turn direction
            } else if (current_Heading < target_Heading && turn_Move_Power < 0) { // if greater than target and moving away
                turn_Move_Power = turn_Move_Power * -1; // reverse turn direction
            }

            left_Drive_Power = original_Drive_Power + turn_Move_Power; // updates drive powers to ensure correct direction
            right_Drive_Power = original_Drive_Power - turn_Move_Power;

            left_Drive.setPower(left_Drive_Power);
            right_Drive.setPower(right_Drive_Power);
        }
        else{
            stillDriving = false;

            //reset variables
        }

        return stillDriving;
    }






}
