
package org.firstinspires.ftc.teamcode.OffSeason2019;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Hardware;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Map;



public abstract class Drive_Base_Template {

    // deine hardware mapping
    private HardwareMap hardwareMap;


    //Define Sensors


    //Define Timers


    //Define Status Flags


    //Encoder values


    // Gyro
    public double getGyroHeading(Orientation angles){
        //Formats Gyro reading into Degrees
        double rawHeading = 0.0;
        rawHeading = -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        return rawHeading;
    }

    public double getEncoderDistance() {
        double EncoderDistance = 0.0;


        // Return an encoder
        return EncoderDistance;
    }



    //Move functions potentially setup for auto and manual, pass stick values

    public boolean Forward(double inchForward_Target){

     // Add code for encoder distance finding if implemented in auto
        return true;
    }

    public boolean Backward(double inchBackward_Target){

        // Add code for encoder distance finding if implemented in auto
        return true;
    }

    public boolean Left(double inchLeft_Target){

        // Add code for encoder distance finding if implemented in auto
        return true;
    }

    public boolean Right(double inchRight_Target){

        // Add code for encoder distance finding if implemented in auto
        return true;
    }



    public void resetEncoders() {

    }


    public void stop() {

    }
    private void setPowerAll(double rfPower, double rbPower, double lfPower, double lbPower){

    }

}