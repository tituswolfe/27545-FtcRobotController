/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.DraculaBase;
import org.firstinspires.ftc.teamcode.HardwareBase27545;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="New Auto 27545", group="Auto")
public class NewAuto extends LinearOpMode {

    HardwareBase27545 driveBase = new HardwareBase27545();


    @Override
    public void runOpMode() {
        driveBase.init(hardwareMap, this);
        HeadingHolder.setHeading(0.0);
        while(opModeInInit()){
            telemetry.addLine("Field heading: " + driveBase.getFieldHeading());
            telemetry.addLine("Press play to start auto.");
            telemetry.update();
        }
        //waitForStart();
        driveBase.imu.resetYaw();
        
        
        driveBase.bucket.setPosition(driveBase.bucketUp);
        
            telemetry.update();

        //displayDiagnostics();
        driveBase.tankDrive(0.5, -5);

       driveBase.DriveSideways(0.6, driveBase.rightDistanceToWall() - 4);

        driveBase.gyroTurn(0.6, 145 + 180);
        driveBase.DriveSideways(0.5, 2);

       driveBase.score();
       
       // pickup
       driveBase.gyroTurn(0.6, 90);
       
      driveBase.tankDrive(0.5, 26.25 - driveBase.rearDistanceToWall());
      driveBase.gyroTurn(0.6, 90);
      while(driveBase.frontRight.isBusy()){}
      driveBase.DriveSideways(0.5, driveBase.rightDistanceToWall() - 48.25);
      driveBase.gyroTurn(0.6, 90);
      while(driveBase.frontRight.isBusy()){}
       
       driveBase.bucket.setPosition(driveBase.bucketDown);
       driveBase.flipper.setPosition(driveBase.flipperOut2);
       driveBase.intake.setPower(0.6);
       driveBase.flipper.setPosition(driveBase.flipperOut);
    //   driveBase.tankDrive(0.5, -2.0);
    //     while(driveBase.frontRight.isBusy()){
          
    //   }
       sleep(2100);
       driveBase.bucket.setPosition(driveBase.bucketWayDown);
       sleep(300);
       driveBase.flipper.setPosition(driveBase.flipperIn);
       sleep(1500);
       driveBase.intake.setPower(0.0);
       driveBase.bucket.setPosition(driveBase.bucketUp);
       
       sleep(300);
       
       driveBase.DriveSideways(0.5, driveBase.rightDistanceToWall() - 12.0);
       driveBase.tankDrive(0.5, -23);
       driveBase.DriveSideways(0.5, driveBase.rightDistanceToWall() - 4);
       driveBase.gyroTurn(0.6, 147 + 180);
        driveBase.DriveSideways(0.5, 5);
     
       driveBase.score();
       
        driveBase.gyroTurn(0.6, 150 + 180);
       //driveBase.tankDrive(.6,-48);
       driveBase.gyroTurn(0.6, 0);
       
       
       
    }
}
