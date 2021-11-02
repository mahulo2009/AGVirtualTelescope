//
// Created by mhuertas on 21/10/21.
//

#ifndef AGVIRTUALTELESCOPE_AGKINEMATIC_H
#define AGVIRTUALTELESCOPE_AGKINEMATIC_H

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

        class AGKinematic {

        public:

            AGKinematic(const ZeroPointParams &zeroPointParams,
                               const KinematicAGParams &kinematicAGParams,
                               const KinematicTelescopeParams &kinematicTelescopeParams);

            void  fromMechanismToFocalPlaneCoordinates(double turnTableAngle, double armAngle,
                                                      double &x, double &y, double &ipd) const;

            void fromFocalPlaneToMechanismCoordinates(double x, double y,
                                                      double &turnTableAngle1, double &armAngle1,
                                                      double &turnTableAngle2, double &armAngle2) const;

            void fromMechanismToNaturalReferenceFrame(double turnTableAngle, double armAngle,
                                                      double &turnTableAngleNatural, double &armAngleNatural) const;

            void fromNaturalToMechanismReferenceFrame(double turnTableAngle, double armAngle,
                                                      double &turnTableAngleMechanism, double &armAngleMechanism) const;

            void fromMechanismToAgSurfaceCoordinates(double turnTableAngle, double armAngle,
                                                     double &x, double &y, double &ipd) const;

            double projectArmLength(double armAngle) const;

            double projectArmAngle(double armAngle) const;

            void projectPointBetweenSurfaces(double xf, double yf,
                                             double surf1Radius, double surf2Radius,
                                             double &x, double &y) const;

            double projectArmLengthFromFocalPlaneCoordinates(double x, double y) const;

            double computeArmRotation(double x, double y,double prjArmLen) const;

            double computeAlfa(double x, double y,
                               double prjArmLen) const;
        private:

            ZeroPointParams zeroPointParams_;
            KinematicAGParams kinematicAGParams_;
            KinematicTelescopeParams kinematicTelescopeParams_;
        };
    }
}
#endif //AGVIRTUALTELESCOPE_AGKINEMATIC_H
