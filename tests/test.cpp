//
// Created by mhuertas on 25/10/21.
//

#include "gtest/gtest.h"
#include <math.h>
#include "AGVirtualTelescope.h"

/*
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



    double x = 111.71602351014417;
    double y = 9.4226915159555951;


    double x,y,ipd;
    vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
    vt.fromMechanismPositionToFocalPlaneCoordinates(turnTableAngle, armAngle, x, y, ipd);


    ASSERT_DOUBLE_EQ(x,111.71602351014417);
    ASSERT_DOUBLE_EQ(y,9.4226915159555951);

    double turnTableAngleExpeted,armAngleExpeted;
    vt.fromFocalPlaneCoordinatesToMechanismPosition(double turnTableAngleExpeted,armAngleExpeted;);

    //ASSERT_DOUBLE_EQ(turnTableAngleExpeted,turnTableAngle);
    ASSERT_DOUBLE_EQ(armAngleExpeted,armAngle);


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







}
*/
class AGVirtualTelescopeTest : public ::testing::Test {
public:

    AGVirtualTelescopeTest() {
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

        vt = new vt::ag::AGVirtualTelescope(zpParams, agParams, telescopeParams);
    }

    vt::ag::AGVirtualTelescope *vt;
};

TEST_F(AGVirtualTelescopeTest, toNaturalReferenceFrame) {
    double turnTableAngle = 0.17453292519943295;  //10 degrees
    double armAngle = 0.3490658503988659;         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);


    ASSERT_DOUBLE_EQ(turnTableAngleNatural, 1.4782932585327664); //84.69996459656939
    ASSERT_DOUBLE_EQ(armAngleNatural, 0.61393701768152531);      //35.175999999999995
}

TEST_F(AGVirtualTelescopeTest, armAngleProjected) {
    double turnTableAngle = 0.17453292519943295;  //10 degrees
    double armAngle = 0.3490658503988659;         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armAngleProjected = vt->armAngleProjected_(armAngleNatural);

    ASSERT_DOUBLE_EQ(armAngleProjected, 0.61398402925648032); //35.17869356483318

}

TEST_F(AGVirtualTelescopeTest, armLengthProjected) {
    double turnTableAngle = 0.17453292519943295;  //10 degrees
    double armAngle = 0.3490658503988659;         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armLengthProjected = vt->armLengthProjected(armAngleNatural, 256.69762595262483, 0.01413018);

    ASSERT_DOUBLE_EQ(armLengthProjected, 256.68050615085065); //256.68050615085065
}

TEST_F(AGVirtualTelescopeTest, fromMechanismPositionToAgSurfaceCoordinates) {
    double turnTableAngle = 0.17453292519943295;  //10 degrees
    double armAngle = 0.3490658503988659;         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double x, y, ipd;
    vt->fromMechanismPositionToAgSurfaceCoordinates(turnTableAngleNatural, armAngleNatural,
                                                    x, y, ipd);

    ASSERT_DOUBLE_EQ(x, 150.86362447274536);
    ASSERT_DOUBLE_EQ(y, 33.12729394694616);
    ASSERT_DOUBLE_EQ(ipd, 5.8478539706355201); //335.05735172623577
}

TEST_F(AGVirtualTelescopeTest, fromMechanismPositionToFocalPlaneCoordinates) {
    double turnTableAngle = 0.17453292519943295;  //10 degrees
    double armAngle = 0.3490658503988659;         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double x, y, ipd;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural,
                                                     x, y, ipd);

    ASSERT_DOUBLE_EQ(x, 150.81368359493527);
    ASSERT_DOUBLE_EQ(y, 33.116327710751285);
    ASSERT_DOUBLE_EQ(ipd, 5.8478539706355201); //335.05735172623577
}


TEST_F(AGVirtualTelescopeTest, fromFocalPlaneCoordinatesToMechanismPosition) {

    double x = 150.81368359493527;
    double y = 33.116327710751285;
    double turnTableAngle, armAngle;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y, turnTableAngle, armAngle);

    //ASSERT_DOUBLE_EQ(turnTableAngle,150.81368359493527);
    ASSERT_DOUBLE_EQ(armAngle, 0.61393701768152531);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
