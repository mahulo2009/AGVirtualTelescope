//
// Created by mhuertas on 21/10/21.
//
#include "AGKinematic.h"

#include <valarray>
#include <gsl/gsl_poly.h>

vt::ag::AGKinematic::AGKinematic(const ZeroPointParams &zeroPointParams,
                                 const KinematicAGParams &kinematicAGParams,
                                 const KinematicTelescopeParams &kinematicTelescopeParams,
                                 bool flipFocalPlane) :
        zeroPointParams_(zeroPointParams),
        kinematicAGParams_(kinematicAGParams),
        kinematicTelescopeParams_(kinematicTelescopeParams),
        flipFocalPlane_(flipFocalPlane) {}

void
vt::ag::AGKinematic::projectPointBetweenSurfaces(double xf, double yf,
                                                 double surf1Radius, double surf2Radius,
                                                 double &x, double &y) const {
    //convenience variables
    double xf_sq = pow(xf,2);
    double yf_sq = pow(yf ,2);
    double surf1Radius_sq = pow(surf1Radius,2);
    double surf2Radius_sq = pow(surf2Radius,2);
    double zf = surf1Radius - sqrt(surf1Radius_sq - (xf_sq + yf_sq));

    //polynomial coeficients
    double a = xf_sq + yf_sq + pow(zf - kinematicTelescopeParams_.m2Distance,2);
    double b = 2 * (zf - kinematicTelescopeParams_.m2Distance) * (kinematicTelescopeParams_.m2Distance - surf2Radius);
    double c = pow(kinematicTelescopeParams_.m2Distance,2) - 2*surf2Radius * kinematicTelescopeParams_.m2Distance;

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






//-------------------------Inverse Kinematic

void vt::ag::AGKinematic::projectFromFocalPlaneSurfaceToArmRotation
        (double xs, double ys, double &x, double &y)
{

    projectPointBetweenSurfaces(xs, ys,
                                kinematicTelescopeParams_.focalPlaneCurvatureRadius,
                                kinematicAGParams_.armRotationSurfaceRadius,
                                x, y);

}

double
vt::ag::AGKinematic::fromFocalPlaneCoordinatesComputeArmProjectedLength
        (double x, double y) const {

    //square of the major semi-axis
    double maxLen_sq = pow(kinematicAGParams_.agArmLength, 2);
    //square of the minor semi-axis
    double minLen_sq = pow(kinematicAGParams_.agArmLength * cos(kinematicAGParams_.agArmTilt), 2);
    //square of the turntable radius
    double rt_sq = pow(kinematicAGParams_.agTurntableRadius, 2);
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
vt::ag::AGKinematic::computeArmRotation
        (double x, double y, double prjArmLen) const {


    //todo add special cases
    double polarRadius = sqrt(pow(x, 2) + pow(y, 2));
    if (polarRadius == 0) {
        return 0.0;
    }

    double rt_sq = pow(kinematicAGParams_.agTurntableRadius, 2);
    double prjArmLen_sq = pow(prjArmLen,2);
    double polarRadius_sq = pow(polarRadius,2);

    double cosRotAngle = (rt_sq + prjArmLen_sq - polarRadius_sq) /
                            (2 * prjArmLen * kinematicAGParams_.agTurntableRadius);

    double armProjectedAngle = acos(cosRotAngle);

    double armRotationAngle = atan(tan(armProjectedAngle) * cos(kinematicAGParams_.agArmTilt));

    return armRotationAngle;
}

double vt::ag::AGKinematic::computeAlfa_(double x, double y, double prjArmLen) const {

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

void
vt::ag::AGKinematic::inverse(double x, double y,
                             double &turnTableAngle1, double &armAngle1,
                             double &turnTableAngle2, double &armAngle2,
                             double &focus)  {

    if(flipFocalPlane_)
        x=-x;

    double xs, ys;
    projectFromFocalPlaneSurfaceToArmRotation(x, y, xs, ys);

    double armProjectedLength = fromFocalPlaneCoordinatesComputeArmProjectedLength(xs, ys);

    double armAngle = computeArmRotation(xs, ys,armProjectedLength);

    double turnTableAngle = computeAlfa_(xs,ys,armProjectedLength);

    //todo take in consideration special cases.
    //first solution
    turnTableAngle1 =  atan2 (y, x) + turnTableAngle;
    armAngle1 = armAngle;

    //second solution
    turnTableAngle2 = atan2 (y, x) - turnTableAngle;
    armAngle2 = -armAngle;

    //todo
    focus = 0;
}



//-------------------------Direct Kinematic

void vt::ag::AGKinematic::toNaturalReferenceFrame(double turnTableAngle, double armAngle,
                                                  double &turnTableAngleNatural, double &armAngleNatural) const
{

    turnTableAngleNatural = zeroPointParams_.asgTurntableOffset +
                            turnTableAngle * zeroPointParams_.asgTurntableSense;
    armAngleNatural = zeroPointParams_.asgArmOffset  +
                      armAngle  * zeroPointParams_.asgArmSense;
}

void vt::ag::AGKinematic::toMechanismReferenceFrame(double turnTableAngle, double armAngle,
                                                    double &turnTableAngleMechanism,
                                                    double &armAngleMechanism) const
{

    turnTableAngleMechanism = zeroPointParams_.asgTurntableOffset  +
                              turnTableAngle * zeroPointParams_.asgTurntableSense;
    armAngleMechanism = zeroPointParams_.asgArmOffset +
                        armAngle * zeroPointParams_.asgArmSense;
}

void vt::ag::AGKinematic::direct(double turnTableAngle, double armAngle,
                                 double &x, double &y, double &ipd)
{

    double turnTableAngleNatural, armAngleNatural;
    toNaturalReferenceFrame(turnTableAngle,armAngle,turnTableAngleNatural, armAngleNatural);

    double armAngleProj = armAngleProjected(armAngleNatural);
    double armLengthProj = armLengthProjected(armAngleProj);

    fromMechanismToAGFocalPlaneFrame(turnTableAngleNatural,
                                     armAngleProj,
                                     armLengthProj, x, y, ipd);

    double xa,ya;
    misalignmentCorrectionCenterRotationAndTurnTabel(x,y,xa,ya);

    double xp,yp;
    projectFromArmRotationToFocalPlaneSurface(xa, ya, xp, yp);

    x=xp;
    y=yp;

}

void vt::ag::AGKinematic::projectFromArmRotationToFocalPlaneSurface
        (double xs, double ys, double &x, double &y)
{

    projectPointBetweenSurfaces(xs, ys,
                                kinematicAGParams_.armRotationSurfaceRadius,
                                kinematicTelescopeParams_.focalPlaneCurvatureRadius,
                                x, y);

}

double vt::ag::AGKinematic::armLengthProjected(double armAngle)
{

    //square of the major semi-axis
    double maxLen_sq = pow(kinematicAGParams_.agArmLength, 2);
    //square of the minor semi-axis
    double minLen_sq = pow(kinematicAGParams_.agArmLength * cos(kinematicAGParams_.agArmTilt), 2);

    //compute the arm projected length
    double armProjectedLength = sqrt(maxLen_sq * pow(sin(armAngle), 2) +
                                     minLen_sq * pow(cos(armAngle), 2));
    return armProjectedLength;

}

double vt::ag::AGKinematic::armAngleProjected(double armAngle)
{

    double armProjectedAngle = atan(tan(armAngle) / cos(kinematicAGParams_.agArmTilt));

    return armProjectedAngle;

}

void vt::ag::AGKinematic::misalignmentCorrectionTurnTabelAndCenterRotation(double x, double y, double &xa, double &ya)
{
    //adjust by the misalignment between the center of rotation of the turntable
    //and the center of the rotation
    xa = x - kinematicAGParams_.turntableRotationCenterX;
    ya = y - kinematicAGParams_.turntableRotationCenterY;
}

void vt::ag::AGKinematic::misalignmentCorrectionCenterRotationAndTurnTabel(double x, double y, double &xa, double &ya)
{

    //adjust by the misalignment between the center of rotation of the turntable
    //and the center of the rotation
    xa = x + kinematicAGParams_.turntableRotationCenterX;
    ya = y + kinematicAGParams_.turntableRotationCenterY;

}

void vt::ag::AGKinematic::fromMechanismToAGFocalPlaneFrame
    (double turnTableAngle, double armAngle,double armLengthProjected,double &x, double &y, double &ipd)
{

    //angle of the instrument frame in relation to the telescope focal plane frame
    ipd = turnTableAngle + armAngle + M_PI;

    //current focal plane coordinates of the detector center
    x = kinematicAGParams_.agTurntableRadius * cos(turnTableAngle) + armLengthProjected * cos(ipd);
    y = kinematicAGParams_.agTurntableRadius * sin(turnTableAngle) + armLengthProjected * sin(ipd);

    if (flipFocalPlane_) //todo be carefull where to do this.
        x = -x;

}