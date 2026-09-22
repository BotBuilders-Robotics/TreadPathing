package org.treadpathing.holonomic;

import org.treadpathing.geometry.MathUtil;
import org.treadpathing.spline.SplinePath;

/**
 * Where the nose points along a path, and how fast it is turning to get there.
 *
 * <p>This interface is the whole difference between the two drivetrains, stated in code. On a
 * tank robot heading is not a free parameter -- it <b>is</b> the direction of travel, so there
 * is nothing to interpolate and the library has no interpolator. A mecanum robot can point
 * anywhere while going anywhere, so somebody has to say what the nose does, and every plan
 * below is a legitimate answer.
 *
 * <p>Both methods are in terms of arc length rather than time, on purpose. The turn rate that
 * matters when planning is radians per <b>inch</b> travelled: multiply by the speed and you
 * get the omega the wheels have to find room for, which is how the generator can solve for a
 * feasible speed in closed form instead of iterating.
 */
public interface HeadingPlan {

    /** Robot heading at this distance along the path, in radians. */
    double headingAt(SplinePath path, double arcLength);

    /** Rate of heading change with distance, radians per inch. */
    double rateAt(SplinePath path, double arcLength);

    String describe();
}
