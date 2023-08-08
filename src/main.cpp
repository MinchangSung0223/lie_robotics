#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include  <pybind11/chrono.h>
#include "lie_robotics.h"
#include <iostream>
#include <vector>
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)
using namespace lr;
using namespace std;
namespace py = pybind11;

PYBIND11_MODULE(lie_robotics, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";
  m.def("se3ToVec", [](const se3& T) {
        return lr::se3ToVec(T);
    });
  m.def("ad", [](const Vector6d& V) {
        return lr::ad(V);
    });
   m.def("VecTose3", [](const Vector6d& V) {
        return VecTose3(V);
    });
   m.def("VecToso3", [](const Vector3d& omg) {
        return VecToso3(omg);
    });
   m.def("so3ToVec", [](const so3& so3mat) {
        return so3ToVec(so3mat);
    });
   m.def("NearZero", [](const double val) {
        return NearZero(val);
    });

   m.def("MatrixExp3", [](const so3& so3mat) {
        return MatrixExp3(so3mat);
    });

   m.def("MatrixExp6", [](const se3& se3mat) {
        return MatrixExp6(se3mat);
    });

   m.def("FKinSpace", [](const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
        return FKinSpace(M,Slist,thetaList);
    });
   m.def("Adjoint", [](const SE3& T) {
        return Ad(T);
    });

   m.def("TransToP", [](const SE3& T) {
        return TransToP(T);
    });
   m.def("TransToR", [](const SE3& T) {
        return TransToR(T);
    });    
   m.def("JacobianSpace", [](const ScrewList& Slist, const JVec& thetaList) {
        return JacobianSpace(Slist,thetaList);
    });    
    
   m.def("JacobianBody", [](const ScrewList& Blist, const JVec& thetaList) {
        return JacobianBody(Blist,thetaList);
    });    

   m.def("MatrixLog3", [](const SO3& R) {
        return MatrixLog3(R);
    });    

   m.def("MatrixLog6", [](const SE3& T) {
        return MatrixLog6(T);
    });    
   m.def("RpToTrans", [](const Matrix3d& R, const Vector3d& p) {
        return RpToTrans(R,p);
    });    
   m.def("FKinBody", [](const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
        return FKinBody(M,Blist,thetaList);
    });    

   m.def("IKinBody", [](const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
        return IKinBody(Blist,M,T,thetalist,eomg,ev);
    });    
   m.def("IKinSpace", [](const ScrewList& Slist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
        return IKinSpace(Slist,M,T,thetalist,eomg,ev);
    });   
    m.def("InverseDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist);
    });   
   m.def("InverseDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return InverseDynamics(thetalist,dthetalist,ddthetalist,g,Ftip,Mlist,Glist,Slist,eef_mass);
    });   

   m.def("GravityForces", [](const JVec& thetalist, const Vector3d& g,
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return GravityForces(thetalist,g,Mlist,Glist,Slist);
    });  

   m.def("GravityForces", [](const JVec& thetalist, const Vector3d& g,
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return GravityForces(thetalist,g,Mlist,Glist,Slist,eef_mass);
    });  

   m.def("MassMatrix", [](const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return MassMatrix(thetalist,Mlist,Glist,Slist);
    });  
   m.def("MassMatrix", [](const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return MassMatrix(thetalist,Mlist,Glist,Slist,eef_mass);
    });  

   m.def("VelQuadraticForces", [](const JVec& thetalist, const JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist);
    });  
    m.def("VelQuadraticForces", [](const JVec& thetalist, const JVec& dthetalist,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist,eef_mass);
    });  
    m.def("EndEffectorForces", [](const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist);
    });  
    m.def("EndEffectorForces", [](const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return EndEffectorForces(thetalist,Ftip,Mlist,Glist,Slist,eef_mass);
    });  
    m.def("ForwardDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
        return ForwardDynamics(thetalist,dthetalist,taulist,g, Ftip,Mlist,Glist,Slist );
    });  
    m.def("ForwardDynamics", [](const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist,double eef_mass) {
        return ForwardDynamics(thetalist,dthetalist,taulist,g, Ftip,Mlist,Glist,Slist ,eef_mass);
    }); 

    m.def("EulerStep", [](JVec& thetalist, JVec& dthetalist, const JVec& ddthetalist, double dt) {
        return EulerStep(thetalist,dthetalist,ddthetalist,dt);
    });  

    m.def("QuinticTimeScalingKinematics", [](double s0,double sT,double ds0,double dsT,double dds0,double ddsT,double Tf, double t) {
        return QuinticTimeScalingKinematics(s0,sT,ds0,dsT,dds0,ddsT,Tf,t);
    });  
      m.def("TransInv", [](const SE3& transform) {
        return TransInv(transform);
    });    


m.def("FKinBody", [](SE3 M,ScrewList Blist, const JVec& q ,const JVec& dq, SE3 &T, Jacobian &Jb,Jacobian& dJb) {
        return lr::FKinBody(M,Blist,q,dq,T,Jb,dJb);
    });
m.def("dJacobianBody", [](SE3 M,ScrewList Blist, const JVec& q ,const JVec& dq) {
        return lr::dJacobianBody(M,Blist,q,dq);
    });
m.def("dexp3", [](const Vector3d& xi) {
        return lr::dexp3(xi);
    });

m.def("dlog3", [](const Vector3d& xi) {
        return lr::dlog3(xi);
    });



m.def("dexp6", [](const Vector6d& lambda) {
        return lr::dexp6(lambda);
    });

m.def("ddexp3", [](const Vector3d& xi, const Vector3d& dxi) {
        return lr::ddexp3(xi,dxi);
    });

m.def("dddexp3", [](const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
        return lr::dddexp3(xi,dxi,y,dy);
    });

m.def("ddexp6", [](const Vector6d& lambda, const Vector6d& lambda_dot) {
        return lr::ddexp6(lambda,lambda_dot);
    });

m.def("skew_sum", [](const Vector3d& a, const Vector3d& b) {
        return lr::skew_sum(a,b);
    });

m.def("ddlog3", [](const Vector3d& xi, const Vector3d& dxi) {
        return lr::ddlog3(xi,dxi);
    });

m.def("dddlog3", [](const Vector3d& xi, const Vector3d& dxi, const Vector3d& y, const Vector3d& dy) {
        return lr::dddlog3(xi,dxi,y,dy);
    });

m.def("dlog6", [](const Vector6d& lambda) {
        return lr::dlog6(lambda);
    });

m.def("ddlog6", [](const Vector6d& lambda, const Vector6d& lambda_dot) {
        return lr::ddlog6(lambda,lambda_dot);
    });
m.def("LieScrewTrajectory", [](const SE3 X0,const SE3 XT,const Vector6d V0,const Vector6d VT,const Vector6d dV0,const Vector6d dVT,double Tf,int N) 
    -> std::tuple<py::list, py::list, py::list> {
        vector<SE3> Xd_list;
        Xd_list.resize(N);
        vector<Vector6d> Vd_list;
        Vd_list.resize(N);
        vector<Vector6d> dVd_list;
        dVd_list.resize(N);
        lr::LieScrewTrajectory(X0,XT,V0,VT,dV0,dVT,Tf,N,Xd_list,Vd_list,dVd_list);
         py::list py_Xd_list;
        py::list py_Vd_list;
        py::list py_dVd_list;

        for (const auto& Xd : Xd_list) {
            py_Xd_list.append(py::cast(Xd));
        }

        for (const auto& Vd : Vd_list) {
            py_Vd_list.append(py::cast(Vd));
        }

        for (const auto& dVd : dVd_list) {
            py_dVd_list.append(py::cast(dVd));
        }

        return std::make_tuple(py_Xd_list, py_Vd_list, py_dVd_list);
    });


    

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
