//
// Created by mhuertas on 21/10/21.
//
#include "AGVirtualTelescope.h"

#include <valarray>
#include <gsl/gsl_poly.h>

vt::ag::AGVirtualTelescope::AGVirtualTelescope(const ZeroPointParams &zeroPointParams,
                                               const KinematicAGParams &kinematicAGParams,
                                               const KinematicTelescopeParams &kinematicTelescopeParams) :
        zeroPointParams_(zeroPointParams),
        kinematicAGParams_(kinematicAGParams),
        kinematicTelescopeParams_(kinematicTelescopeParams) {}

void vt::ag::AGVirtualTelescope::toNaturalReferenceFrame(double turnTableAngle, double armAngle,
                                                         double &turnTableAngleNatural, double &armAngleNatural) const {

    turnTableAngleNatural = (turnTableAngle + zeroPointParams_.asgTurntableOffset) * zeroPointParams_.asgTurntableSense;
    armAngleNatural = (armAngle + zeroPointParams_.asgArmOffset) * zeroPointParams_.asgArmSense;

}

void
vt::ag::AGVirtualTelescope::fromMechanismPositionToAgSurfaceCoordinates(double turnTableAngle, double armAngle,
                                                                        double &x, double &y, double &ipd) {

    //compute the arm projected length
    double armProjectedLength = armLengthProjected(armAngle, kinematicAGParams_.agArmLength,
                                                   kinematicAGParams_.agArmTilt);
    double armAngleProjected = armAngleProjected_(armAngle);

    //angle of the instrument frame in relation to the telescope focal plane frame
    ipd = turnTableAngle + armAngleProjected + M_PI;

    //current focal plane coordinates of the detector center
    x = kinematicAGParams_.agTurntableRadius * cos(turnTableAngle) + armProjectedLength * cos(ipd);
    y = kinematicAGParams_.agTurntableRadius * sin(turnTableAngle) + armProjectedLength * sin(ipd);

    //adjust by the misalignment between the center of rotation of the turntable
    //and the center of the rotation
    x += kinematicAGParams_.turntableRotationCenterX;
    y += kinematicAGParams_.turntableRotationCenterY;

    //take into account the rotation of the field
    //produced by the rotation of the camera
    ipd = turnTableAngle + 2 * armAngleProjected + M_PI;

    //take into account the displacement of the center of rotation
    //of the arm respect to the centre of the image
    double offset_center_x =
            kinematicAGParams_.agOriginRotationRadius *
            cos(turnTableAngle + 2 * armAngleProjected + kinematicAGParams_.agOriginRotationPhase);
    double offset_center_y =
            kinematicAGParams_.agOriginRotationRadius *
            sin(turnTableAngle + 2 * armAngleProjected + kinematicAGParams_.agOriginRotationPhase);

    //todo check this
    x = -x;

    x += offset_center_x;
    y += offset_center_y;

}


double vt::ag::AGVirtualTelescope::armLengthProjected(double armAngle, double armLength, double armTilt) {

    //square of the major semi-axis
    double maxLen_sq = pow(armLength, 2);
    //square of the minor semi-axis
    double minLen_sq = pow(armLength * cos(armTilt), 2);

    //arm projected angle
    double armProjectedAngle = armAngleProjected_(armAngle);

    //compute the arm projected length
    double armProjectedLength = sqrt(maxLen_sq * pow(sin(armProjectedAngle), 2) +
                                     minLen_sq * pow(cos(armProjectedAngle), 2));

    return armProjectedLength;
}

double vt::ag::AGVirtualTelescope::armAngleProjected_(double armAngle) {
    double armProjectedAngle = atan(tan(armAngle) / cos(kinematicAGParams_.agArmTilt));

    return armProjectedAngle;
}

void
vt::ag::AGVirtualTelescope::projectPointBetweenSurfaces(double xf, double yf,
                                                        double surf1Radius, double surf2Radius,
                                                        double &x, double &y) const {
    //convenience variables
    double xf_sq = xf * xf;
    double yf_sq = yf * yf;
    double surf1Radius_sq = surf1Radius * surf1Radius;
    double surf2Radius_sq = surf2Radius * surf2Radius;
    double zf = surf1Radius - sqrt(surf1Radius_sq - (xf_sq + yf_sq));

    //polynomial coeficients
    double a =
            xf_sq + yf_sq + (zf - kinematicTelescopeParams_.m2Distance) * (zf - kinematicTelescopeParams_.m2Distance);
    double b = 2 * (zf - kinematicTelescopeParams_.m2Distance) * (kinematicTelescopeParams_.m2Distance - surf2Radius);
    double c = kinematicTelescopeParams_.m2Distance * kinematicTelescopeParams_.m2Distance
               - 2 * surf2Radius * kinematicTelescopeParams_.m2Distance;

    //solve the equation
    double t, t1, t2;
    int nsols = gsl_poly_solve_quadratic(a, b, c, &t1, &t2);
    if (nsols == 0) {
        throw "Unable to project point ";
    }

    //choose the apropiate solution, always the greatest one
    t = (nsols == 2) ? t2 : t1;

    //project the point
    x = xf * t;
    y = yf * t;

}

void vt::ag::AGVirtualTelescope::fromMechanismPositionToFocalPlaneCoordinates(double turnTableAngle, double armAngle,
                                                                              double &x, double &y, double &ipd) {

    double xs, ys;
    fromMechanismPositionToAgSurfaceCoordinates(turnTableAngle, armAngle, xs, ys, ipd);

    projectPointBetweenSurfaces(xs, ys,
                                kinematicAGParams_.armRotationSurfaceRadius,
                                kinematicAGParams_.focalPlaneCurvatureRadius,
                                x, y);

}

double
vt::ag::AGVirtualTelescope::fromFocalPlaneCoordinatesComputeArmProjectedLength
        (double x, double y, double armLength, double armTilt, double agTurntableRadius) const {

    //square of the major semi-axis
    double maxLen_sq = pow(armLength, 2);
    //square of the minor semi-axis
    double minLen_sq = pow(armLength * cos(armTilt), 2);
    //square of the turntable radius
    double rt_sq = pow(agTurntableRadius, 2);
    //square of the radial component of the point (x,y) coordinate
    double rxy_sq = pow(x, 2) + pow(y, 2);

    //polynomial coefficients
    //coefficient of the 4th degree term
    double a = minLen_sq - maxLen_sq - 4 * rt_sq;
    //coefficient of the 2th degree term
    double b = 2 * (minLen_sq * (rt_sq - rxy_sq) + maxLen_sq * (rt_sq + rxy_sq));
    //independent coefficient
    double c = (minLen_sq - maxLen_sq) * (rt_sq - rxy_sq) * (rt_sq - rxy_sq);

    double discr = b * b - 4 * a * c;
    if (discr < 0) {
        throw "Unexpetedly negative value of the discriminant ";
    }
    double rad = (-b - sqrt(discr)) / (2 * a);
    if (rad < 0) {
        throw "Unexpetedly negative value of the radicant";
    }

    double prjLen = sqrt(rad);
    return prjLen;
}

double
vt::ag::AGVirtualTelescope::computeArmRotation
        (double x, double y, double prjArmLen, double agTurntableRadius, double armRotatorTilt) const {


    //todo add special cases
    double polarRadius = sqrt(pow(x, 2) + pow(y, 2));
    if (polarRadius == 0) {
        return 0.0;
    }

    double rt_sq = pow(agTurntableRadius, 2);
    double prjArmLen_sq = prjArmLen * prjArmLen;
    double polarRadius_sq = polarRadius * polarRadius;

    double cosRotAngle = (rt_sq + prjArmLen_sq - polarRadius_sq) / (2 * prjArmLen * agTurntableRadius);

    double armProjectedAngle = acos(cosRotAngle);

    double armRotationAngle = atan(tan(armProjectedAngle) * cos(armRotatorTilt));

    return armRotationAngle;
}

void
vt::ag::AGVirtualTelescope::fromFocalPlaneCoordinatesToMechanismPosition(double x, double y,
                                                                         double &turnTableAngle1, double &armAngle1,
                                                                         double &turnTableAngle2, double &armAngle2) const {

    //todo check this.
    x=-x;

    double xs, ys;
    projectPointBetweenSurfaces(x, y,
                                kinematicAGParams_.focalPlaneCurvatureRadius,
                                kinematicAGParams_.armRotationSurfaceRadius,
                                xs, ys);


    double armProjectedLength = fromFocalPlaneCoordinatesComputeArmProjectedLength(xs, ys,
                                                                                   kinematicAGParams_.agArmLength,
                                                                                   kinematicAGParams_.agArmTilt,
                                                                                   kinematicAGParams_.agTurntableRadius);


    double armAngle = computeArmRotation(xs, ys, armProjectedLength, kinematicAGParams_.agTurntableRadius,
                                  kinematicAGParams_.agArmTilt);

    double turnTableAngle = computeAlfa_(xs,ys,armProjectedLength);

    //todo take in consideration special cases.
    //first solution
    turnTableAngle1 =  atan2 (y, x) + turnTableAngle;
    armAngle1 = armAngle;

    //second solution
    turnTableAngle2 = atan2 (y, x) - turnTableAngle;
    armAngle2 = -armAngle;
}

double vt::ag::AGVirtualTelescope::computeAlfa_(double x, double y, double prjArmLen) const {

    //todo add special cases
    double polarRadius =  sqrt (pow(x,2) + pow(y,2));

    //as in the previous method, the alfa angle is found using the cosinus law
    double rt_sq = pow(kinematicAGParams_.agTurntableRadius,2);
    double prjArmLen_sq = pow(prjArmLen,2);
    double polarRadius_sq = polarRadius*polarRadius;

    double cosAlfa = (rt_sq + polarRadius_sq - prjArmLen_sq )/(2*polarRadius*kinematicAGParams_.agTurntableRadius);


    double alfa = acos(cosAlfa);

    return alfa;
}

