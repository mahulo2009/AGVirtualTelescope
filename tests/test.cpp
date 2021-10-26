//
// Created by mhuertas on 25/10/21.
//

#include "gtest/gtest.h"
#include <math.h>
#include "AGVirtualTelescope.h"

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

    ASSERT_DOUBLE_EQ(x, -152.26762915843653);
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

    ASSERT_DOUBLE_EQ(x, -152.21632305456839);
    ASSERT_DOUBLE_EQ(y, 33.116131808325456);
    ASSERT_DOUBLE_EQ(ipd, 5.8478539706355201); //335.05735172623577
}


TEST_F(AGVirtualTelescopeTest, fromFocalPlaneCoordinatesToMechanismPosition) {

    double x = -152.21632305456839;
    double y = 33.116131808325456;
    double turnTableAngle1, armAngle1,turnTableAngle2, armAngle2;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                     turnTableAngle1, armAngle1,
                                                     turnTableAngle2, armAngle2);

    ASSERT_DOUBLE_EQ(turnTableAngle1,1.4784755795345474);
    ASSERT_DOUBLE_EQ(armAngle1, 0.61720197338030602);

    ASSERT_DOUBLE_EQ(turnTableAngle2,-1.050032702903593);
    ASSERT_DOUBLE_EQ(armAngle2, -0.61720197338030602);
}

TEST_F(AGVirtualTelescopeTest, one_direction) {

    double x = 150;
    double y = 33;

    double turnTableAngle1, armAngle1,turnTableAngle2, armAngle2;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                     turnTableAngle1, armAngle1,
                                                     turnTableAngle2, armAngle2);

    double x1, y1, ipd;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngle1, armAngle1,
                                                     x1, y1, ipd);

    double x2, y2;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngle1, armAngle1,
                                                     x2, y2, ipd);


    ASSERT_NEAR(x,x1,0.5 );
    ASSERT_NEAR(y,y1,0.8 );

    ASSERT_NEAR(x,x2,0.5 );
    ASSERT_NEAR(y,y2,0.8 );
}

TEST_F(AGVirtualTelescopeTest, other_direction) {

    double turnTableAngle1 = 35 * (M_PI)/180;
    double armAngle1 = 12 * (M_PI)/180;

    double x, y,ipd;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngle1, armAngle1,
                                                     x, y, ipd);


    double turnTableAngleOut1, armAngleOut1,turnTableAngleOut2, armAngleOut2;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                     turnTableAngleOut1, armAngleOut1,
                                                     turnTableAngleOut2, armAngleOut2);


    ASSERT_NEAR(turnTableAngle1,turnTableAngleOut1,0.01 );
    ASSERT_NEAR(armAngle1,armAngleOut1,0.01 );
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}