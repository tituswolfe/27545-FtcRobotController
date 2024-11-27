package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    public void positionTo2(double xTarg, double yTarg, double desiredHeading, double driveSpeed) {
        odometryComputer.bulkUpdate();

        double headingThreshold=1.5, closeEnough=1.5,
                distanceToTarget,directionToTarget,headingError,Kp=.1;
        double v,vx,vy,vStrafeRobot,vForwardRobot;
        double currentX,currentY,currentHeading;

        Pose2DGobilda pos = odometryComputer.getPosition();
        currentX=pos.getX(DistanceUnit.INCH);
        currentY=pos.getY(DistanceUnit.INCH);

        distanceToTarget=Math.hypot(Math.abs(xTarg - currentX),
                Math.abs(yTarg - currentY));
        currentHeading=getOdoFieldHeading();
        headingError= desiredHeading-currentHeading;
        // now loop and drive toward the target pose
        while(Math.abs(distanceToTarget) > closeEnough || Math.abs(headingError)>headingThreshold) {
            if(!((LinearOpMode) callingOpMode).opModeIsActive()) {
                return;
            }
            odometryComputer.bulkUpdate();
            pos = odometryComputer.getPosition();// update position for robot
            currentX=pos.getX(DistanceUnit.INCH);
            currentY=pos.getY(DistanceUnit.INCH);
            currentHeading=getOdoFieldHeading(); // was radians
            headingError= desiredHeading-currentHeading;

            distanceToTarget=Math.hypot(Math.abs(yTarg - currentY),
                    Math.abs(xTarg - currentX));

            v=Math.min(driveSpeed,distanceToTarget*Kp);
            v=Math.max(v,.1);
           // v *= 0.1;
            // v is the magnitude of the velocity vector we want the robot
            // to execute on the field. This will be v until it gets
            // close, then it will proportionately decrease until it slows

            // to a minimum of .1, (so it keeps moving until it gets close
            // enough)
          // directionToTarget=Math.atan2(xTarg-currentX,yTarg-currentY);
             directionToTarget=Math.atan2(yTarg-currentY,xTarg-currentX);
            // this is the angle on the field in radians ccw from the field
            // X axis. Now we have the magnitude and direction we want to
            // drive on the field, we can figure out the forward and
            // strafing powers to send to the mecum drive

            // first lets calculate the x and y components of the field
            // velocity vector... for this we need the direction to the
            // target
            vx= v*Math.cos(directionToTarget);
            vy= v*Math.sin(directionToTarget);
            // now let’s do the coordinate transformation to the robot
            // coordinate system.. which is rotated relative to the field by the
            // current heading
            vStrafeRobot=vx*Math.sin(currentHeading)-vy*Math.cos(currentHeading);
            vForwardRobot=vx*Math.cos(currentHeading)+vy*Math.sin(currentHeading);
            r = Turn(driveSpeed,desiredHeading);
            
            // send these motor powers to the mecanum drive
            applyMecPower2(-vStrafeRobot,-vForwardRobot,r);
            
        } // end of the while loop
        applyMecPower2(0,0,0);
        
    } // end of the method... target position is reached.


    public void positionToFieldCentric(double targetPosX, double targetPosY, double targetHeading, double posThreshold, double headingThreshold, double maxPower, double slowDistanceX, double slowDistanceY) {
        odometryComputer.bulkUpdate();
        double heading = odometryComputer.getPosition().getHeading(AngleUnit.DEGREES);
        if (heading < 0) {
            heading = heading + 360.;
        }
        if (heading > 360) {
            heading = heading - 360.;
        }
        double distanceX = targetPosX - odometryComputer.getPosition().getX(DistanceUnit.INCH);
        double distanceY = targetPosY - odometryComputer.getPosition().getY(DistanceUnit.INCH);
        double headingError = targetHeading - heading;

        double speedX;
        double speedY;
        double speedR;

        double cosTheta;
        double sinTheta;

        double fieldX;
        double fieldY;

        double max;

        double frontLeftWheelMotorPower;
        double frontRightWheelMotorPower;
        double backLeftWheelMotorPower;
        double backRightWheelMotorPower;

        while (
                Math.abs(headingError) > headingThreshold ||
                        Math.abs(distanceX) > posThreshold ||
                        Math.abs(distanceY) > posThreshold
        ) {
            odometryComputer.bulkUpdate();

            // try {
            //     sleep(50);
            // } catch (InterruptedException _) {
            // }

            // try {
            //     Thread.sleep(50);
            // } catch (InterruptedException e) {
            //     throw new RuntimeException(e);
            // }

            if(!((LinearOpMode) callingOpMode).opModeIsActive()){
                return;
            }
            heading = odometryComputer.getPosition().getHeading(AngleUnit.DEGREES);

            if (heading < 0) {
                heading = heading + 360.;
            }
            if (heading > 360) {
                heading = heading - 360.;
            }
            distanceX = targetPosX - odometryComputer.getPosition().getX(DistanceUnit.INCH);
            distanceY = targetPosY - odometryComputer.getPosition().getY(DistanceUnit.INCH);
            headingError = targetHeading - heading;

            speedX = Math.signum(distanceX);
            speedY = Math.signum(distanceY);
            speedR = headingError > 180.0 ? Math.signum(headingError - 360.0) : Math.signum(headingError + 360.0);

            // if (Math.abs(distanceX) < slowDistanceX) {
            //     speedX *= Math.abs(distanceX) / slowDistanceX;
            // }

            // if (Math.abs(distanceY) < slowDistanceY) {
            //     speedY *= Math.abs(distanceY) / slowDistanceY;
            // }

            if (Math.abs(headingError) < 50.) {
                speedR *= Math.abs(headingError) / 50.;
            }

            cosTheta = Math.cos(heading);
            sinTheta = Math.sin(heading);

            fieldX = speedX * cosTheta - speedY * sinTheta;
            fieldY = speedX * sinTheta + speedY * cosTheta;

            frontLeftWheelMotorPower = fieldY + speedR + fieldX;
            frontRightWheelMotorPower = fieldY - speedR - fieldX;
            backLeftWheelMotorPower = fieldY + speedR - fieldX;
            backRightWheelMotorPower = fieldY - speedR + fieldX;

            max = 1.0;
            max = Math.max(max, Math.abs(frontLeftWheelMotorPower));
            max = Math.max(max, Math.abs(frontRightWheelMotorPower));
            max = Math.max(max, Math.abs(backLeftWheelMotorPower));
            max = Math.max(max, Math.abs(backRightWheelMotorPower));

            frontLeftWheelMotorPower /= max;
            frontRightWheelMotorPower /= max;
            backLeftWheelMotorPower /= max;
            backRightWheelMotorPower /= max;

            // if ((Math.abs(frontLeftWheelMotorPower) > maxPower || Math.abs(frontRightWheelMotorPower)  > maxPower || Math.abs(backLeftWheelMotorPower)  > maxPower || Math.abs(backRightWheelMotorPower)  > maxPower)  ||
            //         (frontLeftWheelMotorPower < maxPower && frontRightWheelMotorPower < maxPower && backLeftWheelMotorPower < maxPower && backRightWheelMotorPower < maxPower)
            // ) {
            //     max = Math.max(max, Math.abs(frontLeftWheelMotorPower));
            //     max = Math.max(max, Math.abs(frontRightWheelMotorPower));
            //     max = Math.max(max, Math.abs(backLeftWheelMotorPower));
            //     max = Math.max(max, Math.abs(backRightWheelMotorPower));

            //     double scaleFactor = maxPower / max;
            //     frontLeftWheelMotorPower *= scaleFactor;
            //     frontRightWheelMotorPower *= scaleFactor;
            //     backLeftWheelMotorPower *= scaleFactor;
            //     backRightWheelMotorPower *= scaleFactor;
            // }

            setWheelMotorPower(frontLeftWheelMotorPower, frontRightWheelMotorPower, backLeftWheelMotorPower, backRightWheelMotorPower);
        }

        stopMotors();
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
