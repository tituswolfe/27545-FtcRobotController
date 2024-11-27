package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;
import org.firstinspires.ftc.teamcode.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.gobilda.Pose2DGobilda;

import java.util.Locale;

@TeleOp(group = "Test", name = "Odo")
public class OdometryTest extends LinearOpMode9808 {
    //GoBildaPinpointDriver odometryComputer;
    double oldTime = 0;

    @Override
    protected void pre_init_9808() {
        driveBase.init(hardwareMap,this);
    }

    @Override
    protected void init_9808() {

    }

    @Override
     protected void run_9808() {
        intiOdometryComputer();
    }

    private void intiOdometryComputer(){
        // odometryComputer = driveBase.getHardwareMap().get(GoBildaPinpointDriver.class,"odo");
        // odometryComputer.setOffsets(-84.0, -168.0); // in mm
        // odometryComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        // odometryComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        driveBase.odometryComputer.resetPosAndIMU();
        telemetry.addLine("X offset: " + driveBase.odometryComputer.getXOffset());
        telemetry.addLine("Y offset: " + driveBase.odometryComputer.getYOffset());
        telemetry.update();

        while(opModeIsActive()) {
            driveBase.odometryComputer.bulkUpdate();
            if (gamepad1.a) {
                driveBase.odometryComputer.resetPosAndIMU();
            }
            if (gamepad1.b) {
                driveBase.odometryComputer.recalibrateIMU();
            }
            if (gamepad1.x){
                //driveBase.positionToFieldCentric(0, 5, 0, 2, 5, 0.5, 6, 6);
                //driveBase.positionTo2(0, 5, 0, 0.3);
                //sleep(3000);
                //driveBase.positionTo2(-16.9, 5.5, -46.4, 0.3);
                
                
                driveBase.positionTo2(12, 8, -45, 0.5);
                
                // driveBase.positionTo2(0, 4, 0, 0.2);
                // sleep(1000);
                // driveBase.positionTo2(-14, 4, 0, 0.2);
                // sleep(1000);
                // driveBase.positionTo2(-14, 4, 45, 0.2);
                
                //driveBase.positionTo2(-14.3, 9.6, 45, 0.5);
                //sleep(5000);
                //driveBase.positionTo2(0, 0, -46.4, 0.3);
            }

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            
            Pose2DGobilda idkWhatToCallthis = driveBase.odometryComputer.getPosition();
            telemetry.addData("Heading", driveBase.getOdoFieldHeading());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", idkWhatToCallthis.getX(DistanceUnit.INCH), idkWhatToCallthis.getY(DistanceUnit.INCH), idkWhatToCallthis.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            Pose2DGobilda vel = driveBase.odometryComputer.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Status", driveBase.odometryComputer.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", driveBase.odometryComputer.getFrequency());
            telemetry.addData("REV Hub Frequency: ", frequency);
            telemetry.update();

        }
    }

    @Override
    protected Alliance getAlliance() {
        return null;
    }


    public double getFieldHeading() {
        return 0.0;
    }

    public void turnFieldCentric(double speed, double headingAngle){

    }
}
