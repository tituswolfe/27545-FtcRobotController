package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.HeadingHolder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.IntoTheDeepBase;
import org.firstinspires.ftc.teamcode.LinearOpMode9808;


abstract public class IntoTheDeepAuto extends IntoTheDeepBase {
    //TODO: Defile init operations
    /*
    ============================================================================================
    Init Controls:
        A:                      enableDiagnosticMode()


    ============================================================================================
     */
    abstract protected Position getPosition();
    @Override
    protected void pre_initialize() {

    }

    @Override
    protected void initialize() {
        //driveBase.setLED( getPosition().getStaticColor() );
    }

    /**
     * Run AUTO opmode
     */
    @Override
    protected void run_9808() {
        if( opModeIsActive() ) {
            displayDiagnostics();
            
            driveBase.tankDrive(0.5, 4);

            driveBase.DriveSideways(0.5, driveBase.leftDistanceToWall() - 9);

            driveBase.gyroTurn(0.6, 140);

            driveBase.score();

            finish();
        }
    }

    /**
     * Finalize AUTO phase in prep for Teleop
     */
    private void finish() {
        HeadingHolder.setHeading(driveBase.getFieldHeading());
        setLEDHeartbeat();
        telemetry.addData("Path", "Complete");

        telemetry.update();
        while (opModeIsActive());
    }
}
