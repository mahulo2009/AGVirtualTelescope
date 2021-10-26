//
// Created by mhuertas on 21/10/21.
//

#ifndef AGVIRTUALTELESCOPE_AGVIRTUALTELESCOPE_H
#define AGVIRTUALTELESCOPE_AGVIRTUALTELESCOPE_H

namespace vt {
    namespace ag {

        struct ZeroPointParams {
            double asgTurntableOffset;
            double asgArmOffset;
            short asgTurntableSense;
            short asgArmSense;
        };

        struct KinematicAGParams {
            double agArmLength;
            double agArmTilt;
            double agTurntableRadius;
            double turntableRotationCenterX;
            double turntableRotationCenterY;

            double focalPlaneCurvatureRadius;
            double armRotationSurfaceRadius;

            double agOriginRotationRadius;
            double agOriginRotationPhase;
        };

        struct KinematicTelescopeParams {
            double m2Distance;
        };

        class AGVirtualTelescope {

        public:

            AGVirtualTelescope(const ZeroPointParams &zeroPointParams,
                               const KinematicAGParams &kinematicAGParams,
                               const KinematicTelescopeParams &kinematicTelescopeParams);

            void fromMechanismPositionToFocalPlaneCoordinates(double turnTableAngle, double armAngle,
                                                              double &x, double &y, double &ipd);

            void fromFocalPlaneCoordinatesToMechanismPosition(double x, double y,
                                                              double &turnTableAngle1, double &armAngle1,
                                                              double &turnTableAngle2, double &armAngle2) const;

            void toNaturalReferenceFrame(double turnTableAngle, double armAngle,
                                         double &turnTableAngleNatural, double &armAngleNatural) const;

            double armLengthProjected(double armAngle, double armLength, double armTilt);

            void fromMechanismPositionToAgSurfaceCoordinates(double turnTableAngle, double armAngle,
                                                             double &x, double &y, double &ipd);

            void projectPointBetweenSurfaces(double xf, double yf,
                                             double surf1Radius, double surf2Radius,
                                             double &x, double &y) const;

            double fromFocalPlaneCoordinatesComputeArmProjectedLength(double x, double y,
                                                                      double armLength,
                                                                      double armTilt,
                                                                      double agTurntableRadius) const;

            double computeArmRotation(double x, double y,
                                      double prjArmLen, double agTurntableRadius, double armRotatorTilt) const;

            double computeAlfa_(double x, double y,
                                double prjArmLen) const;

            double armAngleProjected_(double armAngle);

        private:


            ZeroPointParams zeroPointParams_;
            KinematicAGParams kinematicAGParams_;
            KinematicTelescopeParams kinematicTelescopeParams_;
        };
    }
}
#endif //AGVIRTUALTELESCOPE_AGVIRTUALTELESCOPE_H
