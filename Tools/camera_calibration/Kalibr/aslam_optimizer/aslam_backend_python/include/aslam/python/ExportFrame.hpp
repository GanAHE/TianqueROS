#ifndef ASLAM_PYTHON_EXPORT_FRAME_HPP
#define ASLAM_PYTHON_EXPORT_FRAME_HPP
#include <sstream>
#include <aslam/Frame.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/CovarianceReprojectionError.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
//#include <aslam/backend/ReprojectionIntrinsicsError.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/Scalar.hpp>

namespace aslam {
  namespace python {
    
    

    template<int D,typename DESCRIPTOR_T>
    void exportKeypoint(const std::string & descriptorName)
    {
      using namespace boost::python;
      using namespace aslam;
      
      typedef aslam::Keypoint<D,DESCRIPTOR_T> keypoint_t;
      typedef DESCRIPTOR_T descriptor_t;

      std::stringstream str;
      str << "Keypoint" << D << descriptorName;

      void (keypoint_t::*setDescriptor)(const descriptor_t & ) = &keypoint_t::setDescriptor;
	  const descriptor_t & (keypoint_t::*getDescriptor)() const  = &keypoint_t::descriptor;
      
      class_<keypoint_t, bases<KeypointBase> >( str.str().c_str(), init<>() )	
	// 	    const measurement_t & measurement() const;
	.def("measurement",&keypoint_t::measurement,return_value_policy<copy_const_reference>())
	// const measurement_t & y() const;
	.def("y",&keypoint_t::y,return_value_policy<copy_const_reference>())
	// void setMeasurement(const measurement_t & m);
	.def("setMeasurement",&keypoint_t::setMeasurement)
	// const inverse_covariance_t & inverseMeasurementCovariance() const;
	.def("inverseMeasurementCovariance",&keypoint_t::inverseMeasurementCovariance,return_value_policy<copy_const_reference>())
	// const inverse_covariance_t & invR() const;
	.def("invR",&keypoint_t::invR,return_value_policy<copy_const_reference>())
	// void setInverseCovariance(const inverse_covariance_t & invR);
	.def("setInverseCovariance",&keypoint_t::setInverseCovariance)
	// const descriptor_t & descriptor() const;
	.def("descriptor",getDescriptor,return_value_policy<copy_const_reference>())
	// void setDescriptor(const descriptor_t & descriptor);
	.def("setDescriptor",setDescriptor)
	// void setDescriptor(const unsigned char * descriptorData);

	;

    }

    template<typename CAMERA_GEOMETRY_T, typename DESCRIPTOR_T>
    void exportReprojectionError(const std::string & name)
    {
      using namespace boost::python;
      using namespace aslam;
      using namespace aslam::backend;
      typedef CAMERA_GEOMETRY_T geometry_t;
      typedef DESCRIPTOR_T descriptor_t;
      typedef Frame<geometry_t, descriptor_t> frame_t;
      typedef typename frame_t::keypoint_t keypoint_t;

      class_< ReprojectionError<frame_t>, boost::shared_ptr<ReprojectionError<frame_t> >, bases< ErrorTerm > >( name.c_str(),
    		  init<const frame_t * , int ,HomogeneousExpression, CameraDesignVariable<geometry_t> >( (name + "( frame, keypointIndex, homogeneousPointExpression, CameraDesignVariable)").c_str()) )
			;

      class_< SimpleReprojectionError<frame_t>, boost::shared_ptr<SimpleReprojectionError<frame_t> >, bases< ErrorTerm > >( (name + "Simple").c_str(),
    		  init<const frame_t * , int ,HomogeneousExpression >( (name + "Simple( frame, keypointIndex, homogeneousPointExpression )").c_str()) )
			;

    }


    template<typename CAMERA_GEOMETRY_T, typename DESCRIPTOR_T>
    void exportCovarianceReprojectionError(const std::string & name)
    {
      using namespace boost::python;
      using namespace aslam;
      using namespace aslam::backend;
      typedef CAMERA_GEOMETRY_T geometry_t;
      typedef DESCRIPTOR_T descriptor_t;
      typedef Frame<geometry_t, descriptor_t> frame_t;
      typedef typename frame_t::keypoint_t keypoint_t;

      class_< CovarianceReprojectionError<frame_t>, boost::shared_ptr<CovarianceReprojectionError<frame_t> >, bases< ErrorTerm > >( name.c_str(),
    		  init<const frame_t * , int ,HomogeneousExpression, CameraDesignVariable<geometry_t>, aslam::splines::BSplinePoseDesignVariable*, aslam::backend::Scalar* >( (name + "( frame, keypointIndex, homogeneousPointExpression, CameraDesignVariable, bsplineDesignVariable, lineDelayDv)").c_str()) )
			.def("observationTime", &CovarianceReprojectionError<frame_t>::observationTime)
			.def("covarianceMatrix",  &CovarianceReprojectionError<frame_t>::covarianceMatrix)
					;


    }

    // // export the optimizable intrinsics reprojection error:
    // template<typename CAMERA_GEOMETRY_T, typename DESCRIPTOR_T>
	// void exportReprojectionIntrinsicsError(const std::string & name)
	// {
	//   using namespace boost::python;
	//   using namespace aslam;
	//   using namespace aslam::backend;
	//   typedef CAMERA_GEOMETRY_T geometry_t;
	//   typedef DESCRIPTOR_T descriptor_t;
	//   typedef Frame<geometry_t, descriptor_t> frame_t;
	//   typedef typename frame_t::keypoint_t keypoint_t;

	//   class_< ReprojectionIntrinsicsError<frame_t>, boost::shared_ptr<ReprojectionIntrinsicsError<frame_t> >, bases< ErrorTerm > >( name.c_str(), init<>() )
	// .def(init<const frame_t * , int ,HomogeneousExpression >( (name + "( frame, keypointIndex, homogeneousPointExpression)").c_str()));

	// }

    template<typename CAMERA_GEOMETRY_T, typename DESCRIPTOR_T>
    void exportFrame(const std::string & name)
    {
      using namespace boost::python;
      using namespace aslam;
      typedef CAMERA_GEOMETRY_T geometry_t;
      typedef DESCRIPTOR_T descriptor_t;
      typedef Frame<geometry_t, descriptor_t> frame_t;
      typedef typename frame_t::keypoint_t keypoint_t;

      keypoint_t & (frame_t::*keypoint)(size_t) = &frame_t::keypoint;
      
      void (frame_t::*addKeypointPtr)(const keypoint_t & keypoint) = &frame_t::addKeypoint;

      class_< frame_t, boost::shared_ptr<frame_t>, bases<FrameBase> >( name.c_str(), init<>() )
          .def("geometry", &frame_t::geometryPtr)
          .def("setGeometry", &frame_t::setGeometry)
          .def("clearKeypoints", &frame_t::clearKeypoints)
          // const keypoint_t & keypoint(size_t i) const;
          // keypoint_t & keypoint(size_t i);
          .def("keypoint", keypoint, return_internal_reference<>())
          .def("addKeypoint", addKeypointPtr)
          // void addKeypoints(const keypoint_vector_t & keypoints);
          ;

      exportReprojectionError<geometry_t, descriptor_t>(name + "ReprojectionError");

    }







  } // namespace python
} // namespace aslam


#endif /* ASLAM_PYTHON_EXPORT_FRAME_HPP */
