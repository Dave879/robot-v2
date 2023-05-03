#include "data_formatter.h"

DataFormatter::DataFormatter() : graph_index_(0), heatmap_index_(0), pkt_idx_(0)
{
}

std::string DataFormatter::AddBarGraph(std::string name)
{
	CleanStreamAndIncrementIndex(graph_index_);
	tmp_ss_ << "n:b:" << name;
	return tmp_ss_.str();
}

std::string DataFormatter::AddStringLog(color c)
{
	tmp_ss_.str("");
	tmp_ss_ << "0:s:" << c;
	return tmp_ss_.str();
}

std::string DataFormatter::AddStringLog()
{
	tmp_ss_.str("");
	tmp_ss_ << "0:s";
	return tmp_ss_.str();
}

std::string DataFormatter::AddPacketIndex()
{
	tmp_ss_.str("");
	tmp_ss_ << "0:i";
	return tmp_ss_.str();
}

std::string DataFormatter::AddLineGraph(std::string name)
{
	CleanStreamAndIncrementIndex(graph_index_);
	tmp_ss_ << "n:l:" << name;
	return tmp_ss_.str();
}

std::string DataFormatter::AddLineGraph(std::string name, int64_t min, int64_t max)
{
	CleanStreamAndIncrementIndex(graph_index_);
	tmp_ss_ << "n:l:" << name << ":" << min << ":" << max;
	return tmp_ss_.str();
}

std::string DataFormatter::AddHeatmap(std::string name, uint16_t sizex, uint16_t sizey, int32_t min, int32_t max)
{
	CleanStreamAndIncrementIndex(heatmap_index_);
	tmp_ss_ << "n:h:" << name << ":" << sizex << ":" << sizey << ":" << min << ":" << max;
	return tmp_ss_.str();
}

void DataFormatter::CleanStreamAndIncrementIndex(uint16_t &idx)
{
	tmp_ss_.str("");
	tmp_ss_ << idx++ << ":";
}

uint64_t DataFormatter::GetPacketIdx(){
	return pkt_idx_++;
}

void DataFormatter::ResetIdx()
{
	graph_index_ = 0;
	heatmap_index_ = 0;
}

DataFormatter::~DataFormatter()
{
}
