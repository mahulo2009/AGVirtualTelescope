//
// Created by mhuertas on 25/10/21.
//

#include "gtest/gtest.h"
#include <math.h>
#include "AGVirtualTelescope.h"

TEST(example, sum_zero)
{

    vt::ag::ZeroPointParams zpParams;
    zpParams.asgTurntableOffset = 1.3037603333333334;
    zpParams.asgArmOffset = 0.26487116728265947;
    zpParams.asgTurntableSense = 1;
    zpParams.asgArmSense = 1;

    vt::ag::KinematicAGParams agParams;
    agParams.agArmLength = 256.69762595262483;
    agParams.agArmTilt = 0.01413018;
    agParams.agTurntableRadius = 256.340;
    agParams.turntableRotationCenterX = 0.018;
    agParams.turntableRotationCenterY = -0.029;

    agParams.focalPlaneCurvatureRadius = 1792.96;
    agParams.armRotationSurfaceRadius = 18139.9;

    agParams.agOriginRotationRadius = 0.84781719;
    agParams.agOriginRotationPhase = -0.159913123;

    vt::ag::KinematicTelescopeParams telescopeParams;
    telescopeParams.m2Distance = 18136.0;

    double xs=30;
    double ys=-83;

    double x,y;
    vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
    vt.projectPointBetweenSurfaces(xs, ys,
                                   agParams.armRotationSurfaceRadius,
                                   agParams.focalPlaneCurvatureRadius,
                                   x, y);

    double xs1,ys1;
    vt.projectPointBetweenSurfaces(x, y,
                                   agParams.focalPlaneCurvatureRadius,
                                   agParams.armRotationSurfaceRadius,
                                   xs1, ys1);

    EXPECT_TRUE((xs-xs1)<1e-9);
    EXPECT_TRUE((ys-ys1)<1e-9);
}

TEST(example, arm)
{

    vt::ag::ZeroPointParams zpParams;
    zpParams.asgTurntableOffset = 1.3037603333333334;
    zpParams.asgArmOffset = 0.26487116728265947;
    zpParams.asgTurntableSense = 1;
    zpParams.asgArmSense = 1;

    vt::ag::KinematicAGParams agParams;
    agParams.agArmLength = 256.69762595262483;
    agParams.agArmTilt = 0.01413018;
    agParams.agTurntableRadius = 256.340;
    agParams.turntableRotationCenterX = 0.018;
    agParams.turntableRotationCenterY = -0.029;

    agParams.focalPlaneCurvatureRadius = 1792.96;
    agParams.armRotationSurfaceRadius = 18139.9;

    agParams.agOriginRotationRadius = 0.84781719;
    agParams.agOriginRotationPhase = -0.159913123;

    vt::ag::KinematicTelescopeParams telescopeParams;
    telescopeParams.m2Distance = 18136.0;

    double turnTableAngle = 10  * (M_PI / 180.0) ;
    double armAngle = 20  * (M_PI / 180.0);

    double x,y,ipd;
    vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
    vt.fromMechanismPositionToFocalPlaneCoordinates(turnTableAngle, armAngle, x, y, ipd);


    ASSERT_DOUBLE_EQ(x,150.81368359493527);
    ASSERT_DOUBLE_EQ(y,33.1163277107513);

    double turnTableAngleExpeted,armAngleExpeted;
    vt.fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                    turnTableAngleExpeted, armAngleExpeted);

    //ASSERT_DOUBLE_EQ(turnTableAngleExpeted,1);
    ASSERT_DOUBLE_EQ(armAngleExpeted,armAngle);



    /*
    double x = 111.71602351014417;
    double y = 9.4226915159555951;


    double x,y,ipd;
    vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
    vt.fromMechanismPositionToFocalPlaneCoordinates(turnTableAngle, armAngle, x, y, ipd);


    ASSERT_DOUBLE_EQ(x,111.71602351014417);
    ASSERT_DOUBLE_EQ(y,9.4226915159555951);
    */
    /*

    double turnTableAngleExpeted,armAngleExpeted;
    vt.fromFocalPlaneCoordinatesToMechanismPosition(double turnTableAngleExpeted,armAngleExpeted;);

    //ASSERT_DOUBLE_EQ(turnTableAngleExpeted,turnTableAngle);
    ASSERT_DOUBLE_EQ(armAngleExpeted,armAngle);

    /*
    double turnTableAngleNatural, armAngleNatural;
    vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
    vt.toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armLengthProjected = vt.fromMechanismPositionComputerArmProjectedLength(armAngleNatural,
                                                                                   agParams.agArmLength,
                                                                                   agParams.agArmTilt);
    ASSERT_DOUBLE_EQ(agParams.agArmLength,256.69762595262483);
    ASSERT_DOUBLE_EQ(armLengthProjected,256.67671136096851);


    //----

    double x1,y1,ipd;
    vt.fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural, x1, y1, ipd);
    ASSERT_DOUBLE_EQ(x,x1);
    ASSERT_DOUBLE_EQ(y,y1);

    double xs1, ys1;
    vt.projectPointBetweenSurfaces(x, y,
                                   agParams.focalPlaneCurvatureRadius,
                                   agParams.armRotationSurfaceRadius,
                                   xs1, ys1);

    ASSERT_DOUBLE_EQ(xs1,111.73550504951764);
    ASSERT_DOUBLE_EQ(ys1,9.4243346870066631);




    double armProjectedLength1 = vt.fromFocalPlaneCoordinatesComputeArmProjectedLength(xs1, ys1,
                                                                                      agParams.agArmLength,
                                                                                      agParams.agArmTilt,
                                                                                      agParams.agTurntableRadius);


    ASSERT_DOUBLE_EQ(armLengthProjected,armProjectedLength1);

*/





}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
