//
// Created by mhuertas on 25/10/21.
//

#include "gtest/gtest.h"
#include <math.h>
#include "AGKinematic.h"

class AGVirtualTelescopeTest : public ::testing::Test {
public:

    AGVirtualTelescopeTest() {

        telescopeParams.m2Distance = 18136.0;
        telescopeParams.focalPlaneCurvatureRadius = 1792.96;

        zpParamsCassegrain.asgTurntableOffset = 3.141592653589793;
        zpParamsCassegrain.asgArmOffset = -0.03184178687338455;
        zpParamsCassegrain.asgTurntableSense = -1;
        zpParamsCassegrain.asgArmSense = -1;

        agParamsCassegrain.agArmLength = 620.7678110384004;
        agParamsCassegrain.agArmTilt = 0.034191000046568915;
        agParamsCassegrain.agTurntableRadius = 620.546;
        agParamsCassegrain.turntableRotationCenterX = 0.243;
        agParamsCassegrain.turntableRotationCenterY = -0.112;
        agParamsCassegrain.armRotationSurfaceRadius = 18142.343566401694;
        agParamsCassegrain.agOriginRotationRadius = 0.0;
        agParamsCassegrain.agOriginRotationPhase = 0.0;

        vt = new vt::ag::AGKinematic(zpParamsCassegrain,
                                     agParamsCassegrain,
                                     telescopeParams,
                                     false);
    }

    vt::ag::AGKinematic *vt;

    vt::ag::KinematicTelescopeParams telescopeParams;
    vt::ag::ZeroPointParams zpParamsCassegrain;
    vt::ag::KinematicAGParams agParamsCassegrain;

    double turnTableAngle120=2.0943951023931957;
    double armAngleMinus16=-0.293641174672534;

    double turnTableAngle60=1.04719755158979;
    double armAngle15=0.261799387799149;

    double turnTableAngleMinus105=-1.8310221855220052;
    double armAngleMinus15=-0.26179938779914791;

    double turnTableAngle285=4.9726148391117988;
    double armAngle13=0.22995760092576337;
};

TEST_F(AGVirtualTelescopeTest, direct_detail) {

    double turnTableAngleNatural, armAngleNatural;
    vt->toMechanismReferenceFrame(turnTableAngle120,
                                  armAngleMinus16,
                                  turnTableAngleNatural,
                                  armAngleNatural);
    ASSERT_NEAR(turnTableAngleNatural, turnTableAngle60,1e-9);
    ASSERT_NEAR(armAngleNatural, armAngle15,1e-9);

    double armAngleProjected = vt->armAngleProjected(armAngleNatural);
    ASSERT_NEAR(armAngleProjected,0.261945581343536,1e-9); //15.008376273085426

    double armLengthProjected = vt->armLengthProjected(armAngleProjected);
    ASSERT_NEAR(armLengthProjected, 620.429336895012,1e-9);

    double x,y,ipd;
    vt->fromMechanismToAGFocalPlaneFrame(turnTableAngleNatural,
                                         armAngleProjected,
                                         armLengthProjected, x, y, ipd);
    ASSERT_NEAR(x,149.781685353334,1e-7);
    ASSERT_NEAR(y,-61.9035888170484,1e-7);
    ASSERT_NEAR(ipd,4.45073578652312,1e-7);

    double xa,ya;
    vt->misalignmentCorrectionCenterRotationAndTurnTabel(x,y,xa,ya);
    ASSERT_NEAR(xa,150.024685353334,1e-7);
    ASSERT_NEAR(ya,-62.0155888170484,1e-7);
    ASSERT_NEAR(ipd,4.45073578652312,1e-7);

    double xp,yp;
    vt->projectFromArmRotationToFocalPlaneSurface(xa,ya,xp,yp);
    ASSERT_NEAR(xp,149.96981748443,1e-7);
    ASSERT_NEAR(yp,-61.9929081282726,1e-7);
    ASSERT_NEAR(ipd,4.45073578652312,1e-7);

}

TEST_F(AGVirtualTelescopeTest, direct) {

    double x,y,ipd;
    vt->direct(turnTableAngle120,
               armAngleMinus16,
               x, y, ipd);

    ASSERT_NEAR(x,149.96981748443,1e-7);
    ASSERT_NEAR(y,-61.9929081282726,1e-7);
    ASSERT_NEAR(ipd,4.45073578652312,1e-7);

}

TEST_F(AGVirtualTelescopeTest, inverse_detail)
{
    double x = 149.96981748443;
    double y = -61.9929081282726;

    double xs, ys;
    vt->projectFromFocalPlaneSurfaceToArmRotation(x, y, xs, ys);

    ASSERT_NEAR(xs,150.0246853533335,1e-7);
    ASSERT_NEAR(ys,-62.01558881704841,1e-7);

    double xa,ya;
    vt->misalignmentCorrectionTurnTabelAndCenterRotation(xs,ys,xa,ya);

    ASSERT_NEAR(xa,149.7816853533335,1e-7);
    ASSERT_NEAR(ya,-61.903588817048409,1e-7);

    double armProjectedLength = vt->fromFocalPlaneCoordinatesComputeArmProjectedLength(xa, ya);
    ASSERT_NEAR(armProjectedLength,620.42933689501194,1e-7);

    double armAngle = vt->computeArmRotation(xa, ya,armProjectedLength);
    ASSERT_NEAR(armAngle,0.26179938779914791,1e-7);

    double turnTableAngle = vt->computeAlfa_(xa,ya,armProjectedLength);
    ASSERT_NEAR(turnTableAngle,1.4391098685558985,1e-7);

    //first solution
    double turnTableNaturaAngle1 = atan2 (ya, xa) + turnTableAngle;
    double armNaturalAngle1 = armAngle;

    ASSERT_NEAR(turnTableNaturaAngle1, turnTableAngle60, 1e-10);
    ASSERT_NEAR(armNaturalAngle1, armAngle15, 1e-10);

    double turnTableAngle1Mechanism, armAngle1Mechanism;
    vt->toMechanismReferenceFrame(turnTableNaturaAngle1,
                                  armNaturalAngle1,
                                  turnTableAngle1Mechanism,
                                  armAngle1Mechanism);

    ASSERT_NEAR(turnTableAngle1Mechanism, turnTableAngle120, 1e-7);
    ASSERT_NEAR(armAngle1Mechanism, armAngleMinus16, 1e-10);

    //second solution
    double turnTableNaturalAngle2 = atan2 (ya, xa) - turnTableAngle;
    double armNaturalAngle2 = -armAngle;

    ASSERT_NEAR(turnTableNaturalAngle2, turnTableAngleMinus105, 1e-7);
    ASSERT_NEAR(armNaturalAngle2, armAngleMinus15, 1e-7);

    double turnTableAngle2Mechanism, armAngle2Mechanism;
    vt->toMechanismReferenceFrame(turnTableNaturalAngle2,
                                  armNaturalAngle2,
                                  turnTableAngle2Mechanism,
                                  armAngle2Mechanism);

    ASSERT_NEAR(turnTableAngle2Mechanism,turnTableAngle285, 1e-10);
    ASSERT_NEAR(armAngle2Mechanism, armAngle13, 1e-10);

}

TEST_F(AGVirtualTelescopeTest, inverse) {

    double x = 149.96981748443;
    double y = -61.9929081282726;

    double turnTableAngle1, armAngle1;
    double turnTableAngle2, armAngle2;
    double focus;

    vt->inverse(x,y,
            turnTableAngle1, armAngle1,
            turnTableAngle2, armAngle2,
            focus);

    //first solution
    ASSERT_NEAR(turnTableAngle1, turnTableAngle60, 1e-10);
    ASSERT_NEAR(armAngle1, armAngle15, 1e-10);

    //second solution
    ASSERT_NEAR(turnTableAngle2, turnTableAngleMinus105, 1e-7);
    ASSERT_NEAR(armAngle2, armAngleMinus15, 1e-7);

    //focus
    ASSERT_NEAR(focus, 6.6160346820288396, 1e-7);

}