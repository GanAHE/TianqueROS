#include <Eigen/Core>

#include <numpy_eigen/boost_python_headers.hpp>
Eigen::Matrix<boost::int64_t, 5, 5> test_long_5_5(const Eigen::Matrix<boost::int64_t, 5, 5> & M)
{
	return M;
}
void export_long_5_5()
{
	boost::python::def("test_long_5_5",test_long_5_5);
}

