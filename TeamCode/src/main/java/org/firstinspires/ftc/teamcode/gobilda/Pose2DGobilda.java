package org.firstinspires.ftc.teamcode.gobilda;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Pose2DGobilda {

    private final double x;
    private final double y;
    private final DistanceUnit distanceUnit;
    private final double heading;
    private final AngleUnit headingUnit;

    public Pose2DGobilda(DistanceUnit distanceUnit, double x, double y, AngleUnit headingUnit, double heading) {
        this.x = x;
        this.y = y;
        this.distanceUnit = distanceUnit;
        this.heading = heading;
        this.headingUnit = headingUnit;
    }

    public double getX(DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, x);
    }

    public double getY(DistanceUnit unit) {
        return unit.fromUnit(this.distanceUnit, y);
    }

    public double getHeading(AngleUnit unit) {
        return unit.fromUnit(this.headingUnit, heading);
    }

}
