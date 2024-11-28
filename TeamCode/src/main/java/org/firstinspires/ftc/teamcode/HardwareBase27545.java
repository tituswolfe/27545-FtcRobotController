package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

public class HardwareBase27545 extends DraculaBase {

    @Override
    void initRobotSpecificMotors() {
        lift = getDcMotor("lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setTargetPosition(0);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slide = getDcMotor("slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setTargetPosition(0);
        slide.setPower(0);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void initRobotSpecificHardware() {
        initOdometryComputer();
    }
    public void initOdometryComputer(){
        odometryComputer = getHardwareMap().get(GoBildaPinpointDriver.class,"odo");
        odometryComputer.setOffsets(-84.0, -168.0);
        odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometryComputer.resetPosAndIMU();
    }

    public void driveTo(double targetXPos, double targetYPos, double targetHeading, double maxSpeed) {
        //autonomous navigation

        double headingThreshold = 1.5;
        double positionThreshold = 1.5;
        double velocityThreshold = 1;

        double proportionalGain = 0.1; // Kp
        double derivativeGain = 0.1; // Kd

        odometryComputer.bulkUpdate();
        Pose2DGobilda pos = odometryComputer.getPosition();
        Pose2DGobilda vel = odometryComputer.getVelocity();

        // Update position
        double currentXPos = pos.getX(DistanceUnit.INCH);
        double currentYPos = pos.getY(DistanceUnit.INCH);

        double distanceToTarget = Math.hypot(
                Math.abs(targetYPos - currentYPos),
                Math.abs(targetXPos - currentXPos)
        );

        // Update heading
        double currentHeading = getOdoFieldHeading();
        double headingError = targetHeading - currentHeading;

        // Update velocity
        double currentXVel = vel.getX(DistanceUnit.INCH); // vel = Inch / sec (according to gobilda)
        double currentYVel = vel.getY(DistanceUnit.INCH);

        double velocityTowardTarget = Math.hypot(
                Math.abs(currentYVel),
                Math.abs(currentXVel)
        );


        double derivative;

        double velocity;

        double directionToTarget;

        double velocityX;
        double velocityY;

        double velocityXRobotCentric;
        double velocityYRobotCentric;

        // TODO: added vel check
        while(Math.abs(distanceToTarget) > positionThreshold || Math.abs(headingError) > headingThreshold || Math.abs(velocityTowardTarget) > velocityThreshold) {
            if(!((LinearOpMode) callingOpMode).opModeIsActive()) {
                return;
            }

            // Get position and velocity
            odometryComputer.bulkUpdate();
            vel = odometryComputer.getVelocity();
            pos = odometryComputer.getPosition();

            // Update position
            currentXPos = pos.getX(DistanceUnit.INCH);
            currentYPos = pos.getY(DistanceUnit.INCH);

            distanceToTarget = Math.hypot(
                    Math.abs(targetYPos - currentYPos),
                    Math.abs(targetXPos - currentXPos)
            );

            // Update heading
            currentHeading = getOdoFieldHeading();
            headingError = targetHeading - currentHeading;

            // Update velocity
            currentXVel = vel.getX(DistanceUnit.INCH); // vel = Inch / sec (according to gobilda)
            currentYVel = vel.getY(DistanceUnit.INCH);

            velocityTowardTarget = Math.hypot(
                    Math.abs(currentYVel),
                    Math.abs(currentXVel)
            );

            // derivative = -1(velocity) * Kd
            derivative = -(velocityTowardTarget * derivativeGain);

            // velocity = proportional + derivative
            // min drive speed < velocity < max drive speed
            velocity = Math.min(maxSpeed, distanceToTarget*proportionalGain + derivative);
            velocity = Math.max(velocity, 0.1);

            // Get theta to target in radians
            directionToTarget = Math.atan2(targetYPos - currentYPos,targetXPos - currentXPos);

            // Calculate directional velocity
            velocityX = velocity * Math.cos(directionToTarget);
            velocityY = velocity * Math.sin(directionToTarget);

            // Calculate strafe w/ directional velocity at current heading
            velocityXRobotCentric=velocityX*Math.sin(currentHeading)-velocityY*Math.cos(currentHeading);
            velocityYRobotCentric=velocityX*Math.cos(currentHeading)+velocityY*Math.sin(currentHeading);
            r = Turn(maxSpeed,targetHeading);

            applyMecPower2(-velocityXRobotCentric,-velocityYRobotCentric, r);
        }
        setWheelMotorPower(0, 0, 0, 0);
        //applyMecPower2(0,0,0);
    }


    public void score(){

        intake.setPower(-0.2);
        bucket.setPosition(bucketUp);
        ((LinearOpMode) callingOpMode).sleep(900);
        intake.setPower(0.0);


        lift.setPower(0.9);
        lift.setTargetPosition(2800);
        //while(lift.isBusy()){ }
        ((LinearOpMode) callingOpMode).sleep(1600);

        //DriveSideways(.3,-2);
        tankDrive(0.15, 3.);


        bucket.setPosition(bucketDown);
        ((LinearOpMode) callingOpMode).sleep(1100);
        bucket.setPosition(bucketWayDown);

        tankDrive(0.5, -4);
        while(frontLeft.isBusy()){
        }
        lift.setPower(.8);

        flipper.setPosition(flipperOut2);

        lift.setTargetPosition(0);
        ((LinearOpMode) callingOpMode).sleep(1550);
        lift.setPower(0);


        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        flipper.setPosition(flipperIn);
    }

    public void scoreTeleop() {
        gyroTurn(0.6, 0);
        DriveSideways(0.5, rightDistanceToWall() - 6);
        tankDrive(0.5, frontDistanceToWall() - 7);

        gyroTurn(0.6, 140 + 180);
        DriveSideways(0.5, 1.5);

        intake.setPower(-0.3);
        bucket.setPosition(bucketUp);
        ((LinearOpMode) callingOpMode).sleep(700);
        intake.setPower(0.0);
        //DriveSideways(0.5, -2.5);
        lift.setPower(0.8);
        lift.setTargetPosition(2800);
        //while(lift.isBusy()){}
        ((LinearOpMode) callingOpMode).sleep(1200);


        tankDrive(0.3, 2);

        bucket.setPosition(bucketDown);
        ((LinearOpMode) callingOpMode).sleep(700);
        bucket.setPosition(bucketWayDown);

        tankDrive(0.5, -4);
        lift.setPower(.8);

        flipper.setPosition(flipperOut2);


        lift.setTargetPosition(0);
        ((LinearOpMode) callingOpMode).sleep(1600);
        lift.setPower(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flipper.setPosition(flipperIn);
    }
}
