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
        zpParams.asgTurntableOffset = 1.8244;
        zpParams.asgArmOffset = -0.03184178687338455;
        zpParams.asgTurntableSense = 1;
        zpParams.asgArmSense = 1;

        vt::ag::KinematicAGParams agParams;
        agParams.agArmLength = 620.7678110384004;
        agParams.agArmTilt = 0.034191000046568915;
        agParams.agTurntableRadius = 620.546;
        agParams.turntableRotationCenterX = 0.243;
        agParams.turntableRotationCenterY = -0.112;

        agParams.focalPlaneCurvatureRadius = 1792.96;
        agParams.armRotationSurfaceRadius = 18142.343566401694;

        agParams.agOriginRotationRadius = 0.0;
        agParams.agOriginRotationPhase = 0.0;

        vt::ag::KinematicTelescopeParams telescopeParams;
        telescopeParams.m2Distance = 18136.0;

        vt = new vt::ag::AGVirtualTelescope(zpParams, agParams, telescopeParams);
    }

    vt::ag::AGVirtualTelescope *vt;
};

TEST_F(AGVirtualTelescopeTest, toNaturalReferenceFrame) {
    double turnTableAngle = 60 * (M_PI/180);  //60 degrees
    double armAngle = 15 * (M_PI/180);        //15 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);


    ASSERT_DOUBLE_EQ(turnTableAngleNatural, 164.53042014366738 * (M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngleNatural, 13.1756 * (M_PI/180));
}

TEST_F(AGVirtualTelescopeTest, armAngleProjected) {
    double turnTableAngle = 60 * (M_PI)/180;
    double armAngle = 15 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armAngleProjected = vt->armAngleProjected_(armAngleNatural);

    ASSERT_NEAR(armAngleProjected, 0.23008738454430308,0.001);

}

TEST_F(AGVirtualTelescopeTest, armLengthProjected) {
    double turnTableAngle = 60 * (M_PI)/180;
    double armAngle = 15 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double armLengthProjected = vt->armLengthProjected(armAngleNatural,
                                                       620.7678110384004,
                                                       0.034191000046568915);

    ASSERT_NEAR(armLengthProjected, 620.42387595872424,0.001);
}

TEST_F(AGVirtualTelescopeTest, fromMechanismPositionToAgSurfaceCoordinates) {
    double turnTableAngle = 60 * (M_PI/180); //10 degrees
    double armAngle = 15 * (M_PI/180);         //20 degrees

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double x, y, ipd;
    vt->fromMechanismPositionToAgSurfaceCoordinates(turnTableAngleNatural, armAngleNatural,
                                                    x, y, ipd);

    ASSERT_NEAR(x, -22.107904647961451,0.0001);
    ASSERT_NEAR(y, 140.65106767157047,0.0001);
    ASSERT_NEAR(ipd, 6.4733649738749968,0.001);
}

TEST_F(AGVirtualTelescopeTest, fromMechanismPositionToFocalPlaneCoordinates) {
    double turnTableAngle = 60 * (M_PI)/180;
    double armAngle = 15 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

    double x, y, ipd;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural,
                                                     x, y, ipd);

    ASSERT_NEAR(x, -22.101687351003488,0.001);
    ASSERT_NEAR(y, 140.6115130656911,0.001);
    ASSERT_NEAR(ipd, 6.4733649738749968,0.001); //335.05735172623577
}


TEST_F(AGVirtualTelescopeTest, fromFocalPlaneCoordinatesToMechanismPosition) {

    double x = -22.101687351003488;
    double y = 140.6115130656911;

    double turnTableAngle1, armAngle1,turnTableAngle2, armAngle2;
    double turnTableAngleMec1,armAngleMec1;
    double turnTableAngleMec2,armAngleMec2;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                     turnTableAngle1, armAngle1,
                                                     turnTableAngle2, armAngle2);

    vt->toMechanismReferenceFrame(turnTableAngle1,armAngle1,turnTableAngleMec1,armAngleMec1);
    vt->toMechanismReferenceFrame(turnTableAngle2,armAngle2,turnTableAngleMec2,armAngleMec2);

    ASSERT_DOUBLE_EQ(turnTableAngleMec1,59.8998189001601 * ( M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngleMec1, 14.99320487793146 * ( M_PI/180) );

    ASSERT_DOUBLE_EQ(turnTableAngleMec2,-106.82628709710077 * ( M_PI/180) );
    ASSERT_DOUBLE_EQ(armAngleMec2, -11.34440487793146 * ( M_PI/180));
}

TEST_F(AGVirtualTelescopeTest, one_direction) {


    double x = -22.101687351003488;
    double y = 140.6115130656911;

    double turnTableAngle1, armAngle1,turnTableAngle2, armAngle2;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                     turnTableAngle1, armAngle1,
                                                     turnTableAngle2, armAngle2);

    //ASSERT_DOUBLE_EQ(turnTableAngle1,-105.43863077271575 * ( M_PI/180) );
    //ASSERT_DOUBLE_EQ(armAngle1, 31.152342231389216 * ( M_PI/180) );

    //ASSERT_DOUBLE_EQ(turnTableAngle2,-254.2569553729316 * ( M_PI/180) );
    //ASSERT_DOUBLE_EQ(armAngle2,-31.152342231389216 * ( M_PI/180));

    double turnTableAngleMechanism1, armAngleMechanism1;
    vt->toMechanismReferenceFrame(turnTableAngle1, armAngle1, turnTableAngleMechanism1, armAngleMechanism1);

    ASSERT_NEAR(turnTableAngleMechanism1,59.8998189001601 * ( M_PI/180) ,0.001);
    ASSERT_NEAR(armAngleMechanism1,14.99320487793146 * ( M_PI/180) ,0.001);

    double turnTableAngleMechanism2, armAngleMechanism2;
    vt->toMechanismReferenceFrame(turnTableAngle2, armAngle2, turnTableAngleMechanism2, armAngleMechanism2);

    ASSERT_NEAR(turnTableAngleMechanism2,-106.82628709710077 * ( M_PI/180),0.001);
    ASSERT_NEAR(armAngleMechanism2, -11.34440487793146 * ( M_PI/180) ,0.001 );


    double turnTableAngleNatural1, armAngleNatural1;
    vt->toNaturalReferenceFrame(turnTableAngleMechanism1, armAngleMechanism1, turnTableAngleNatural1, armAngleNatural1);

    double x1, y1, ipd1;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural1, armAngleNatural1,
                                                     x1, y1, ipd1);
    ASSERT_NEAR(-22.344625436089224,x1,0.001 );
    ASSERT_NEAR(140.4995849483368,y1,0.001 );

    double turnTableAngleNatural2, armAngleNatural2;
    vt->toNaturalReferenceFrame(turnTableAngleMechanism2, armAngleMechanism2, turnTableAngleNatural2, armAngleNatural2);

    double x2, y2, ipd2;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural2, armAngleNatural2,
                                                     x2, y2, ipd2);

    ASSERT_NEAR(-22.344625436089224,x2,0.001 );
    ASSERT_NEAR(140.4995849483368,y2,0.001 );

}

TEST_F(AGVirtualTelescopeTest, other_direction) {

    double turnTableAngle1 = 60 * (M_PI)/180;
    double armAngle1 = 15 * (M_PI)/180;

    double turnTableAngleNatural, armAngleNatural;
    vt->toNaturalReferenceFrame(turnTableAngle1, armAngle1, turnTableAngleNatural, armAngleNatural);

    double x, y,ipd;
    vt->fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural,
                                                     x, y, ipd);

    ASSERT_NEAR(x,-22.101687351003488,0.001 );
    ASSERT_NEAR(y,140.6115130656911,0.001 );

    double turnTableAngleOut1, armAngleOut1,turnTableAngleOut2, armAngleOut2;
    vt->fromFocalPlaneCoordinatesToMechanismPosition(x, y,
                                                     turnTableAngleOut1, armAngleOut1,
                                                     turnTableAngleOut2, armAngleOut2);

    //ASSERT_NEAR(turnTableAngleOut1,164.4302390438275 * ( M_PI/180) ,0.001);
    //ASSERT_NEAR(armAngleOut1, 13.168804877931459 * ( M_PI/180),0.001 );

    //ASSERT_NEAR(turnTableAngleOut2,92.72020805737101 * ( M_PI/180) ,0.001);
    //ASSERT_NEAR(armAngleOut2, 31.152342231389216 * ( M_PI/180),0.001);

    double turnTableAngleMechanism, armAngleMechanism;
    vt->toMechanismReferenceFrame(turnTableAngleOut1, armAngleOut1, turnTableAngleMechanism, armAngleMechanism);

    ASSERT_NEAR(turnTableAngleMechanism,59.8998189001601* ( M_PI/180),0.001);
    ASSERT_NEAR(armAngleMechanism,14.99320487793146 * ( M_PI/180) ,0.001 );


    vt->toMechanismReferenceFrame(turnTableAngleOut2, armAngleOut2, turnTableAngleMechanism, armAngleMechanism);

    ASSERT_NEAR(turnTableAngleMechanism,-106.82628709710077 * ( M_PI/180),0.001);
    ASSERT_NEAR(armAngleMechanism,-11.34440487793146* ( M_PI/180) ,0.001 );

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}