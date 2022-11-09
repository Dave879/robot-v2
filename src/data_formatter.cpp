#include "data_formatter.h"

DataFormatter::DataFormatter() : graph_index_(0)
{
}

std::string DataFormatter::AddBarGraph(std::string name)
{
	tmp_ss_.str("");
	tmp_ss_ << std::setfill('0') << std::setw(sizeof(uint16_t) * 2) << std::hex << graph_index_++ << "nb>" << name;
	return tmp_ss_.str();
}

std::string DataFormatter::AddLineGraph(std::string name)
{
	tmp_ss_.str("");
	tmp_ss_ << std::setfill('0') << std::setw(sizeof(uint16_t) * 2) << std::hex << graph_index_++ << "nl>" << name;
	return tmp_ss_.str();
}

std::string DataFormatter::AddHeatmap(std::string name, uint8_t sizex, uint8_t sizey)
{
	tmp_ss_.str("");
	tmp_ss_ << std::setfill('0') << std::setw(sizeof(uint16_t) * 2) << std::hex << graph_index_++ << "nh" << sizex << "*" << sizey << ">" << name;
	return tmp_ss_.str();
}

void DataFormatter::ResetIdx()
{
	graph_index_ = 0;
}

DataFormatter::~DataFormatter()
{
}
