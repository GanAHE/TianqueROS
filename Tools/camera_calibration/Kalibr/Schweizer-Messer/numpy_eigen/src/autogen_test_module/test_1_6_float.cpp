#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<float, 1, 6> test_float_1_6(const Eigen::Matrix<float, 1, 6> & M)
{
	return M;
}
void export_float_1_6()
{
	boost::python::def("test_float_1_6",test_float_1_6);
}

