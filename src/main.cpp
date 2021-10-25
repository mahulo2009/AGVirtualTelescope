#include <iostream>
#include <cmath>

#include "AGVirtualTelescope.h"


int main(int argc, char *argv[]) {

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

    if (std::stoi(argv[1]) == 1) {

        std::cout << "TurnTable: " << argv[2] << " Arm: " << argv[3] << std::endl;
        std::cout << std::endl;

        double turnTableAngle = std::stod(argv[2]) * (M_PI / 180.0);
        double armAngle = std::stod(argv[3]) * (M_PI / 180.0);

        turnTableAngle -= zpParams.asgTurntableOffset;
        armAngle -= zpParams.asgArmOffset;

        double turnTableAngleNatural, armAngleNatural;
        vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
        vt.toNaturalReferenceFrame(turnTableAngle, armAngle, turnTableAngleNatural, armAngleNatural);

        std::cout << "turnTableAngleNatural: " << turnTableAngleNatural << std::endl;
        std::cout << "armAngleNatural: " << armAngleNatural << std::endl;
        std::cout << std::endl;


        double armLengthProjected = vt.armLengthProjected(armAngleNatural, agParams.agArmLength, agParams.agArmTilt);
        std::cout << "agArmLength: " << agParams.agArmLength
                  << " agArmTilt: " << agParams.agArmTilt
                  << " armLengthProjected: " << armLengthProjected << std::endl;
        std::cout << std::endl;


        double xs, ys, ipd;
        vt.fromMechanismPositionToAgSurfaceCoordinates(turnTableAngleNatural, armAngleNatural, xs, ys, ipd);

        std::cout << "xs: " << xs << std::endl;
        std::cout << "ys: " << ys << std::endl;
        std::cout << "ipds: " << ipd << std::endl;
        std::cout << std::endl;

        double x, y;
        vt.projectPointBetweenSurfaces(xs, ys,
                                       agParams.armRotationSurfaceRadius,
                                       agParams.focalPlaneCurvatureRadius,
                                       x, y);

        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "ipd: " << ipd << std::endl;
        std::cout << std::endl;

        vt.fromMechanismPositionToFocalPlaneCoordinates(turnTableAngleNatural, armAngleNatural, x, y, ipd);
        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "ipd: " << ipd << std::endl;
        std::cout << std::endl;

        return 0;
    } else {

        std::cout << "X: " << argv[1] << " Y: " << argv[2] << std::endl;
        std::cout << std::endl;

        double x = std::stod(argv[2]);
        double y = std::stod(argv[3]);

        vt::ag::AGVirtualTelescope vt(zpParams, agParams, telescopeParams);
        double xs, ys;
        vt.projectPointBetweenSurfaces(x, y,
                                       agParams.focalPlaneCurvatureRadius,
                                       agParams.armRotationSurfaceRadius,
                                       xs, ys);

        std::cout << "xs: " << xs << std::endl;
        std::cout << "ys: " << ys << std::endl;
        std::cout << std::endl;

        double armProjectedLength = vt.fromFocalPlaneCoordinatesComputeArmProjectedLength(xs, ys,
                                                                                          agParams.agArmLength,
                                                                                          agParams.agArmTilt,
                                                                                          agParams.agTurntableRadius);
        std::cout << "armProjectedLength: " << armProjectedLength << std::endl;
        std::cout << std::endl;

        double armRotation = vt.computeArmRotation(xs, ys, armProjectedLength, agParams.agTurntableRadius,
                                                   agParams.agArmTilt);

        std::cout << "armRotation: " << armRotation * (180 / M_PI) << std::endl;
        std::cout << std::endl;

        return 0;
    }

}
