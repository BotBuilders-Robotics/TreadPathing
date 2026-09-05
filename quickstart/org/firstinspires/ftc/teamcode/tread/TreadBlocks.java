package org.firstinspires.ftc.teamcode.tread;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.treadpathing.follower.Follower;
import org.treadpathing.geometry.MathUtil;
import org.treadpathing.geometry.Pose;
import org.treadpathing.route.Route;
import org.treadpathing.route.RouteBuilder;

/**
 * Tread Pathing for the Blocks editor.
 *
 * <p>Blocks cannot hold a builder object and chain calls onto it, so the fluent API is
 * flattened into a sequence of statements against one route that this class keeps for you:
 * <b>start route</b>, then a spline or a turn or a hold, then <b>run route</b>. That is the
 * same order you would write in Java; only the shape differs.
 *
 * <pre>
 *   start route at 9, 60 facing 0
 *   spline to 34, 60 facing 0
 *   spline to 52, 84 facing 70
 *   stop and hold 1.5
 *   run route
 * </pre>
 *
 * <h3>What Blocks cannot do</h3>
 *
 * Markers and actions are missing on purpose. They take a callback, and Blocks has no way to
 * hand one across. If you need something to happen part-way along a route, split it: run the
 * first half, do the thing, run the second. That is clumsier than a marker and it is honest
 * about what is actually possible.
 *
 * <h3>Units</h3>
 *
 * Inches and degrees throughout, because that is what the field is marked in and what a Blocks
 * user will be reading off the planner. The library works in radians internally; the
 * conversion happens here.
 */
public class TreadBlocks extends BlocksOpModeCompanion {

    /**
     * These are static because Blocks gives us nowhere else to put them, which means they
     * outlive the OpMode: the Robot Controller keeps this class loaded between runs. Every
     * entry point therefore has to cope with state left behind by a previous run rather than
     * assuming a clean start. {@link #startRoute} rebuilds both from scratch for that reason.
     */
    private static Follower follower;
    private static RouteBuilder builder;

    private TreadBlocks() {
    }

    // ----- building -----------------------------------------------------------------------

    @ExportToBlocks(
            heading = "start route",
            comment = "Sets the robot's starting pose and begins a new route. Call this first.",
            parameterLabels = {"x (in)", "y (in)", "heading (deg)"},
            parameterDefaultValues = {"9", "60", "0"})
    public static void startRoute(double x, double y, double headingDegrees) {
        // Rebuilt every time. Reusing a follower from a previous run would inherit its pose
        // and its localizer, and the robot would drive from wherever it thought it was.
        follower = Constants.buildFollower(hardwareMap);
        follower.setPose(new Pose(x, y, MathUtil.toRadians(headingDegrees)));
        builder = follower.route();
    }

    @ExportToBlocks(
            heading = "spline to",
            comment = "Adds a smooth curve to this pose. Heading is where the nose points.",
            parameterLabels = {"x (in)", "y (in)", "heading (deg)"},
            parameterDefaultValues = {"34", "60", "0"})
    public static void splineTo(double x, double y, double headingDegrees) {
        require().splineTo(new Pose(x, y, MathUtil.toRadians(headingDegrees)));
    }

    @ExportToBlocks(
            heading = "line to",
            comment = "Drives straight to this point, turning first if it has to.",
            parameterLabels = {"x (in)", "y (in)"},
            parameterDefaultValues = {"34", "60"})
    public static void lineTo(double x, double y) {
        require().lineTo(x, y);
    }

    @ExportToBlocks(
            heading = "turn to",
            comment = "Turns in place to face this heading.",
            parameterLabels = {"heading (deg)"},
            parameterDefaultValues = {"90"})
    public static void turnTo(double headingDegrees) {
        require().turnToDegrees(headingDegrees);
    }

    @ExportToBlocks(
            heading = "stop and hold",
            comment = "Stops, then holds the pose until it is settled or the timeout expires.",
            parameterLabels = {"seconds"},
            parameterDefaultValues = {"1.5"})
    public static void stopAndHold(double seconds) {
        require().stopAndHold(seconds);
    }

    @ExportToBlocks(
            heading = "wait",
            comment = "Sits still for this long.",
            parameterLabels = {"seconds"},
            parameterDefaultValues = {"1"})
    public static void waitSeconds(double seconds) {
        require().waitSeconds(seconds);
    }

    @ExportToBlocks(
            heading = "drive backwards",
            comment = "Whether the following segments are driven in reverse.",
            parameterLabels = {"reversed"},
            parameterDefaultValues = {"true"})
    public static void setReversed(boolean reversed) {
        require().setReversed(reversed);
    }

    // ----- running ------------------------------------------------------------------------

    @ExportToBlocks(
            heading = "run route",
            comment = "Drives the route. Blocks here until it finishes or the OpMode stops.")
    public static void runRoute() {
        Route route = require().build();
        follower.follow(route);
        // opModeIsActive() is what makes the Stop button work. A Blocks user has no way to add
        // this themselves, so it is not optional.
        while (linearOpMode.opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.stopDrive();
        builder = follower.route();
    }

    @ExportToBlocks(
            heading = "is following",
            comment = "True while a route is still running.")
    public static boolean isFollowing() {
        return follower != null && follower.isBusy();
    }

    // ----- where am I ---------------------------------------------------------------------

    @ExportToBlocks(heading = "x", comment = "Current field X, in inches.")
    public static double getX() {
        return follower == null ? 0.0 : follower.getPose().getX();
    }

    @ExportToBlocks(heading = "y", comment = "Current field Y, in inches.")
    public static double getY() {
        return follower == null ? 0.0 : follower.getPose().getY();
    }

    @ExportToBlocks(heading = "heading", comment = "Current heading, in degrees.")
    public static double getHeadingDegrees() {
        return follower == null ? 0.0 : follower.getPose().getHeadingDegrees();
    }

    // ----- internals ----------------------------------------------------------------------

    /**
     * Blocks has no types to stop you calling "spline to" before "start route", so the error
     * has to be a readable sentence at runtime rather than a null pointer.
     */
    private static RouteBuilder require() {
        if (builder == null) {
            throw new IllegalStateException(
                    "Tread: call \"start route\" before adding steps to it.");
        }
        return builder;
    }
}
