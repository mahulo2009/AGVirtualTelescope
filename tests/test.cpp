//
// Created by mhuertas on 25/10/21.
//

#include "gtest/gtest.h"
#include <math.h>
#include "AGKinematic.h"

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

        vt = new vt::ag::AGKinematic(zpParams, agParams, telescopeParams);
    }

    vt::ag::AGKinematic *vt;
};

TEST_F(AGVirtualTelescopeTest, toNaturalReferenceFrame) {
    double turnTableAngle = 10 * (M_PI/180);  //10 degrees
    double armAngle = 20 * (M_PI/180);         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    ASSERT_DOUBLE_EQ(turnTableAngleNatural, 84.69996459656939 * (M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngleNatural, 35.175999999999995 * (M_PI/180));
}

TEST_F(AGVirtualTelescopeTest, armAngleProjected) {
    double turnTableAngle = 150 * (M_PI)/180;
    double armAngle = 33 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armAngleProjected = vt->projectArmAngle(armAngleNatural);

    ASSERT_NEAR(armAngleProjected, 0.840879,0.001);
}

TEST_F(AGVirtualTelescopeTest, armLengthProjected) {
    double turnTableAngle = 150 * (M_PI)/180;
    double armAngle = 33 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armLengthProjected = vt->projectArmLength(armAngleNatural);

    ASSERT_NEAR(armLengthProjected, 256.686,0.001);
}

TEST_F(AGVirtualTelescopeTest, fromMechanismPositionToAgSurfaceCoordinates) {
    double turnTableAngle = 10 * (M_PI/180); //10 degrees
    double armAngle = 20 * (M_PI/180);         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double x, y, ipd;
    vt->fromMechanismToAgSurfaceCoordinates(turnTableAngleNatural, armAngleNatural,
                                                    x, y, ipd);

    ASSERT_NEAR(x, -150.86362447274536,0.0001);
    ASSERT_NEAR(y, 33.12729394694616,0.0001);
    ASSERT_NEAR(ipd, 5.8478539706355201,0.001);
}

TEST_F(AGVirtualTelescopeTest, fromMechanismPositionToFocalPlaneCoordinates) {
    double turnTableAngle = 150 * (M_PI)/180;
    double armAngle = 33 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double x, y, ipd;
    vt->fromMechanismToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural,
                                                     x, y, ipd);

    ASSERT_NEAR(x, 194.396,0.001);
    ASSERT_NEAR(y, 75.3485,0.001);
    ASSERT_NEAR(ipd, 8.74511,0.001); //335.05735172623577
}


TEST_F(AGVirtualTelescopeTest, fromFocalPlaneCoordinatesToMechanismPosition) {

    double x = 194.396;
    double y = 75.3485;

    double turnTableAngle1, armAngle1,turnTableAngle2, armAngle2;
    vt->fromFocalPlaneToMechanismCoordinates(x, y,
                                                     turnTableAngle1, armAngle1,
                                                     turnTableAngle2, armAngle2);

    ASSERT_DOUBLE_EQ(turnTableAngle1,224.90681732897377 * ( M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngle1, 47.98425494373352 * ( M_PI/180) );

    ASSERT_DOUBLE_EQ(turnTableAngle2,92.72020805737101 * ( M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngle2, -47.98425494373352 * ( M_PI/180));
}

TEST_F(AGVirtualTelescopeTest, one_direction) {


    double x = 194.97929980616783;
    double y = 76.007883995849298;

    double turnTableAngle1, armAngle1,turnTableAngle2, armAngle2;
    vt->fromFocalPlaneToMechanismCoordinates(x, y,
                                             turnTableAngle1, armAngle1,
                                             turnTableAngle2, armAngle2);

    ASSERT_DOUBLE_EQ(turnTableAngle1,224.90681732897377 * ( M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngle1, 47.98425494373352 * ( M_PI/180) );

    ASSERT_DOUBLE_EQ(turnTableAngle2,92.72020805737101 * ( M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngle2, -47.98425494373352 * ( M_PI/180));


    double turnTableAngleMechanism1, armAngleMechanism1;
    vt->fromNaturalToMechanismReferenceFrame(turnTableAngle1, armAngle1, turnTableAngleMechanism1, armAngleMechanism1);

    ASSERT_NEAR(turnTableAngleMechanism1,150 * ( M_PI/180),0.00001);
    ASSERT_NEAR(armAngleMechanism1,33 * ( M_PI/180) ,0.00001);

    double turnTableAngleMechanism2, armAngleMechanism2;
    vt->fromNaturalToMechanismReferenceFrame(turnTableAngle2, armAngle2, turnTableAngleMechanism2, armAngleMechanism2);

    ASSERT_NEAR(turnTableAngleMechanism2,18.020152910439034 * ( M_PI/180),0.001);
    ASSERT_NEAR(armAngleMechanism2,-63.16057550404143 * ( M_PI/180) ,0.001 );


    double turnTableAngleNatural1, armAngleNatural1;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngleMechanism1, armAngleMechanism1, turnTableAngleNatural1, armAngleNatural1);

    double x1, y1, ipd1;
    vt->fromMechanismToFocalPlaneCoordinates(turnTableAngleNatural1, armAngleNatural1,
                                                     x1, y1, ipd1);
    ASSERT_NEAR(193.81503109019101,x1,0.001 );
    ASSERT_NEAR(74.687409983179677,y1,0.001 );

    double turnTableAngleNatural2, armAngleNatural2;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngleMechanism2, armAngleMechanism2, turnTableAngleNatural2, armAngleNatural2);

    double x2, y2, ipd2;
    vt->fromMechanismToFocalPlaneCoordinates(turnTableAngleNatural2, armAngleNatural2,
                                                     x2, y2, ipd2);

    ASSERT_NEAR(193.55149531382219,x2,0.001 );
    ASSERT_NEAR(75.137712457803943,y2,0.001 );

}

TEST_F(AGVirtualTelescopeTest, other_direction) {

    double turnTableAngle1 = 150 * (M_PI)/180;
    double armAngle1 = 33 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->fromMechanismToNaturalReferenceFrame(turnTableAngle1, armAngle1, turnTableAngleNatural, armAngleNatural);

    double x, y,ipd;
    vt->fromMechanismToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural,
                                             x, y, ipd);

    ASSERT_NEAR(x,194.97929980616783,0.001 );
    ASSERT_NEAR(y,76.007883995849298,0.001 );


    double turnTableAngleOut1, armAngleOut1,turnTableAngleOut2, armAngleOut2;
    vt->fromFocalPlaneToMechanismCoordinates(x, y,
                                                     turnTableAngleOut1, armAngleOut1,
                                                     turnTableAngleOut2, armAngleOut2);

    ASSERT_NEAR(turnTableAngleOut1,224.90681732897377 * ( M_PI/180) ,0.001);
    ASSERT_NEAR(armAngleOut1, 47.98425494373352 * ( M_PI/180),0.001 );

    ASSERT_NEAR(turnTableAngleOut2,92.72020805737101 * ( M_PI/180) ,0.001);
    ASSERT_NEAR(armAngleOut2, -47.98425494373352 * ( M_PI/180),0.001);

    double turnTableAngleMechanism, armAngleMechanism;
    vt->fromNaturalToMechanismReferenceFrame(turnTableAngleOut1, armAngleOut1, turnTableAngleMechanism, armAngleMechanism);

    ASSERT_NEAR(turnTableAngleMechanism,150.20661557149663 * ( M_PI/180),0.001);
    ASSERT_NEAR(armAngleMechanism,32.80830819432461 * ( M_PI/180) ,0.001 );


    vt->fromNaturalToMechanismReferenceFrame(turnTableAngleOut2, armAngleOut2, turnTableAngleMechanism, armAngleMechanism);

    ASSERT_NEAR(turnTableAngleMechanism,18.020152910439034 * ( M_PI/180),0.001);
    ASSERT_NEAR(armAngleMechanism,-63.16034893797588 * ( M_PI/180) ,0.001 );

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}