package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriverControls;
import org.firstinspires.ftc.teamcode.HeadingHolder;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;

@TeleOp(name = "ITDTeleop", group = "Linear Opmode")

public class IntoTheDeepTeleop extends IntoTheDeepBase {
    //TODO: Defile controller operations
    /*
    ============================================================================================
    Driving Controls:
        Left Stick - Movement (forward, back, left, right)
        Right Stick - Rotation (clockwise, counterclockwise)
        DPad - Slow movement

                             ***METHODS IN IntoTheDeepBase***
        A :                     sweeperIn()
        B :                     sweeperOut()
        X :
        Y :
        Left bumper:
        Right bumper:
        A + Left bumper:
        A + Right bumper:
    ============================================================================================
    */

    private final double DEFAULT_ARM_POWER = 0.1;
    private final double NO_POWER = 0.0;
    private final int ARM_INCREMENT = 20;
    private final int SLIDE_INCREMENT = 20;

    protected double lastSavedAngle = HeadingHolder.getHeading();
    protected DriverControls controller = new DriverControls();
    private boolean isIntakeOut = false;

    @Override
    protected void initialize() {
        // Set diagnostic mode if GP1.a is pressed
        if( gamepad1.a) {
            diagnosticMode = true;
        }

        // Reset the Gyro if GP2.LS is pressed
        controller.resetGyro(driveBase);
        updateTelemetry();
    }

    protected void pre_initialize() {
        //driveBase.arm.setPower( DEFAULT_ARM_POWER );
        //driveBase.slide.setPower( NO_POWER );
        controller.init(this);
    }

    private void updateTelemetry() {
        if (controller.fieldCentric) {
            telemetry.addLine(getColorString() + " TeleOp :  FIELD Centric");
        } else {
            telemetry.addLine(getColorString() + " TeleOp :  ROBOT Centric");
        }

        telemetry.addLine("Driving Speed is HIGH");
        telemetry.addLine("Press Driver 2 Left Stick button at any time to reset the gyro");

        telemetry.addLine("Diagnostic Mode (press A to toggle): " + diagnosticMode);
        telemetry.addData("Gyro initialized to:   ", lastSavedAngle);
        telemetry.addData("heading:   ", driveBase.getFieldHeading());
    }

    @Override
    protected void run_9808() {
        setStaticLED();
        displayDiagnostics();
        while (opModeIsActive()) {
            displayDiagnostics();
            controller.updateSpeedFactor();
            controller.calculateDriveControls();
            controller.calculateDPadCreep();
            controller.updateDriveMode();
            processSweeper();

            // Preset to score
//            if( gamepad1.y) {
//                score(Basket.TOP);
//            }

            if (gamepad1.y) {
                if (!isIntakeOut) {
                    driveBase.scoreTeleop();
                }
            }


            // if(gamepad1.a){
            //     if(isIntakeOut) {
            //         retractIntakeLow();
            //         isIntakeOut = false;
            //     } else {
            //         extendIntakeLow();
            //         isIntakeOut = true;
            //     }
            // }
            
            if(gamepad1.x){
                if(isIntakeOut) {
                    retractIntake();
                    isIntakeOut = false;
                } else {
                    extendIntake();
                    isIntakeOut = true;
                }
            }
            
            if (gamepad1.b) {
                driveBase.imu.resetYaw();
            }
            
            
            if (gamepad1.right_bumper) {
                driveBase.flipper.setPosition(driveBase.flipperOut - 0.18);
            }

            if (gamepad1.right_trigger > 0.1) {
                driveBase.flipper.setPosition(driveBase.flipperOut);
            }
            if (gamepad1.a) {
                driveBase.flipper.setPosition(driveBase.flipperIn);
            }
            
            if(gamepad1.left_bumper) {
                driveBase.intake.setPower(0.4);
                sleep(50);
                
                if (isIntakeOut) {
                    driveBase.intake.setPower(0.4);
                } else {
                    driveBase.intake.setPower(0.0);
                }
            }

            if (gamepad1.left_trigger > 0.1) {
                driveBase.intake.setPower(-0.5);
                sleep(50);
                
                if (isIntakeOut) {
                    driveBase.intake.setPower(0.4);
                } else {
                    driveBase.intake.setPower(0.0);
                }
            }
            
           
            
            // reverse intake during feild
            
            
            // if (gamepad1.left_trigger > 0.1) {
            //     controller.resetGyro(driveBase);
            // }

//            if (gamepad1.b)
//                prepareToTravel();
//
//            if( gamepad1.a) {
//                travel();
//            }//

            processArm();
            //processSlide();
            controller.move(driveBase);
            displayDiagnostics();
            processDevControls();
        }
    }
    
    
    private void processDevControls(){
        if (gamepad2.left_trigger > 0.1) {
            driveBase.lift.setPower(0.5);
            driveBase.lift.setTargetPosition(driveBase.lift.getCurrentPosition() - 30);
            sleep(50);
            
            driveBase.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBase.lift.setTargetPosition(0);
            driveBase.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        if (gamepad2.right_trigger > 0.1) {
            driveBase.slide.setPower(0.5);
            driveBase.slide.setTargetPosition(driveBase.slide.getCurrentPosition() - 30);
            sleep(50);
            
            driveBase.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveBase.slide.setTargetPosition(0);
            driveBase.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }


    private void processArm() {
//        if( gamepad2.left_bumper ) {
//            driveBase.incrementMotorSafe(driveBase.arm,-1 * ARM_INCREMENT, .8, driveBase.armTravelPosition, driveBase.armLowered);
//        } else if (controller.triggered(gamepad2.left_trigger)) {
//            driveBase.incrementMotorSafe(driveBase.arm, ARM_INCREMENT, .8, driveBase.armTravelPosition, driveBase.armLowered);
//        }
    }

    private void extendIntakeLow(){
        driveBase.bucket.setPosition(driveBase.bucketWayDown);
        
        driveBase.slide.setTargetPosition(977);
        driveBase.slide.setPower(0.8);
        sleep(1500);

        driveBase.flipper.setPosition(driveBase.flipperOut);


        driveBase.intake.setPower(0.4);
    }

    private void retractIntakeLow(){
        driveBase.bucket.setPosition(driveBase.bucketWayDown);
        driveBase.flipper.setPosition(driveBase.flipperOut2);
        
        driveBase.slide.setTargetPosition(0);
        driveBase.slide.setPower(0.8);
        
        
        driveBase.tankDrive(0.7, 12);
        while(driveBase.frontRight.isBusy()) {}

        driveBase.flipper.setPosition(driveBase.flipperIn);

        driveBase.bucket.setPosition(driveBase.bucketUp);
        
        driveBase.intake.setPower(0);
    }
    
    
    private void extendIntake(){
        driveBase.bucket.setPosition(driveBase.bucketWayDown);
        sleep(500);
        
        
        driveBase.slide.setTargetPosition(987);
        driveBase.slide.setPower(0.8);
        while(driveBase.slide.isBusy()) {}
        driveBase.slide.setPower(0.0);
        
        driveBase.flipper.setPosition(driveBase.flipperMid);

        driveBase.intake.setPower(0.4);
    }

    private void retractIntake(){
        driveBase.bucket.setPosition(driveBase.bucketWayDown);
        driveBase.flipper.setPosition(driveBase.flipperIn);
        
        driveBase.slide.setTargetPosition(0);
        driveBase.slide.setPower(0.8);
        sleep(1000);
        driveBase.slide.setPower(0.0);
        
        driveBase.bucket.setPosition(driveBase.bucketUpForTravel);
        sleep(600);
        
        driveBase.intake.setPower(0);
    }

    private void retractSlide(){
        driveBase.slide.setTargetPosition(0);
        driveBase.slide.setPower(0.8);
    }

    private void moveSlide() {
        driveBase.slide.setTargetPosition(977);
        driveBase.slide.setPower(0.8);
    }

    private void processSlide() { //  977
        if( gamepad1.right_bumper ) {
            driveBase.incrementMotorSafe(driveBase.slide, 30, 0.5, 0 , 90);
            sleep(30); // 2814 max
        } else if (controller.triggered(gamepad1.right_trigger)) {
            driveBase.incrementMotorSafe(driveBase.slide, -30, 0.5, 0 , 90);
            sleep(30); // 2814 max
            //bvlldriveBase.incrementMotorSafe(driveBase.slide, -1 * SLIDE_INCREMENT, .8, driveBase.slideOut, driveBase.slideIn);
        }
    }

    private void processSweeper() {
//        if (gamepad2.a) {
//            sweeperIn();
//        } else if (gamepad2.b) {
//            sweeperOut();
//        }
//        else {
//            sweeperOff();
//        }
    }
}
