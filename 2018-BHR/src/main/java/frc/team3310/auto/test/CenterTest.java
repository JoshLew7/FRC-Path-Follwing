package frc.team3310.auto.test;

import java.util.ArrayList;

import frc.team3310.auto.PathBuilder;
import frc.team3310.auto.PathBuilder.Waypoint;
import frc.team3310.auto.PathContainer;
import frc.team3310.utility.control.Path;
import frc.team3310.utility.math.RigidTransform2d;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Translation2d;

public class CenterTest implements PathContainer {

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0, 0, 0));
        sWaypoints.add(new Waypoint(-20, 0, 0, 60));
        sWaypoints.add(new Waypoint(-40, 0, 20, 60));
        sWaypoints.add(new Waypoint(-60, 20, 20, 60));
        sWaypoints.add(new Waypoint(-120, 20, 0, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0));
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}