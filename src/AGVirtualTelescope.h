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

            double armRotationSurfaceRadius;

            double agOriginRotationRadius;
            double agOriginRotationPhase;
        };

        struct KinematicTelescopeParams {
            double m2Distance;
            double focalPlaneCurvatureRadius;
        };

        class AGVirtualTelescope {

        public:

            AGVirtualTelescope(const ZeroPointParams &zeroPointParams,
                               const KinematicAGParams &kinematicAGParams,
                               const KinematicTelescopeParams &kinematicTelescopeParams,
                               bool flipFocalPlane);

            void projectPointBetweenSurfaces(double xf, double yf,
                                             double surf1Radius, double surf2Radius,
                                             double &x, double &y) const;




            //-------------------------Inverse Kinematic
            void projectFromFocalPlaneSurfaceToArmRotation(double xs, double ys, double &x, double &y);

            void misalignmentCorrectionTurnTabelAndCenterRotation(double x, double y, double &xa, double &ya);

            double fromFocalPlaneCoordinatesComputeArmProjectedLength(double x, double y) const;

            double computeArmRotation(double x, double y,double prjArmLen) const;

            double computeAlfa_(double x, double y,
                                double prjArmLen) const;

            void toMechanismReferenceFrame(double turnTableAngle, double armAngle,
                                           double &turnTableAngleMechanism, double &armAngleMechanism) const;

            void fromFocalPlaneToMechanism(double x, double y,
                                           double &turnTableAngle1, double &armAngle1,
                                           double &turnTableAngle2, double &armAngle2);




            //-------------------------Direct Kinematic

            void fromMechanismToFocalPlane(double turnTableAngle, double armAngle,
                                                              double &x, double &y, double &ipd);

            void toNaturalReferenceFrame(double turnTableAngle, double armAngle,
                                         double &turnTableAngleNatural, double &armAngleNatural) const;


            void projectFromArmRotationToFocalPlaneSurface(double xs, double ys, double &x, double &y);

            double armAngleProjected(double armAngle);

            double armLengthProjected(double armAngle);

            void misalignmentCorrectionCenterRotationAndTurnTabel(double x, double y, double &xa, double &ya);

            void fromMechanismToAGFocalPlaneFrame(double turnTableAngle, double armAngle,
                                                  double armLengthProjected,
                                                  double &x, double &y, double &ipd);


        private:


            ZeroPointParams zeroPointParams_;
            KinematicAGParams kinematicAGParams_;
            KinematicTelescopeParams kinematicTelescopeParams_;
            bool flipFocalPlane_;
        };
    }
}
#endif //AGVIRTUALTELESCOPE_AGVIRTUALTELESCOPE_H
