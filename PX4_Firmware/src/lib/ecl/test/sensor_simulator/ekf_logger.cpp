#include "ekf_logger.h"

EkfLogger::EkfLogger(std::shared_ptr<Ekf> ekf):
_ekf{ekf},
_ekf_wrapper(ekf)
{

}

EkfLogger::~EkfLogger() {}

void EkfLogger::setFilePath(std::string file_path)
{
	_file_path = file_path;
}

void EkfLogger::writeStateToFile()
{
	if(!_file_opened)
	{
		_file.open(_file_path);
		_file_opened = true;
		_file << "Timestamp";
		if(_state_logging_enabled)
		{
			for(int i = 0; i < 24; i++)
			{
				_file << ",state[" << i << "]";
			}
		}
		if(_variance_logging_enabled)
		{
			for(int i = 0; i < 24; i++)
			{
				_file << ",variance[" << i << "]";
			}
		}
		_file << std::endl;
	}

	if (_file)
	{
		writeState();
	}
	else
	{
		std::cerr << "Can not write to output file" << std::endl;
		std::exit(-1);
	}

}

void EkfLogger::writeState()
{
	if(_state_logging_enabled)
	{
		uint64_t time = _ekf->get_imu_sample_delayed().time_us;
		_file << time;
		if(_state_logging_enabled)
		{
			matrix::Vector<float, 24> state = _ekf->getStateAtFusionHorizonAsVector();
			for(int i = 0; i < 24; i++)
			{
				_file << "," << state(i);
			}
		}
		if(_variance_logging_enabled)
		{
			matrix::Vector<float, 24> variance = _ekf->covariances_diagonal();
			for(int i = 0; i < 24; i++)
			{
				_file << "," << variance(i);
			}
		}
		_file << std::endl;
	}
}
