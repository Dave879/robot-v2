#include "data_formatter.h"

DataFormatter::DataFormatter() : pkt_idx_(0), rep_string_(0)
{
}

std::string DataFormatter::AddBarGraph(const std::string &name)
{
	return "0&n&b&" + name;
}

std::string DataFormatter::AddBarGraph(const std::string &name, const int &id)
{
	sprintf(numstr_, "%i", id);
	return std::string(numstr_) + "&n&b&" + name;
}

std::string DataFormatter::AddStringLog(const color &c)
{
	return "0&s&" + c;
}

std::string DataFormatter::AddStringLog()
{
	return "0&s";
}

std::string DataFormatter::AddPacketIndex()
{
	return "0&i";
}

std::string DataFormatter::AddLineGraph(const std::string &name)
{
	return "0&n&l&" + name;
}

std::string DataFormatter::AddLineGraph(const std::string &name, const int &id)
{
	sprintf(numstr_, "%i", id);
	return std::string(numstr_) + "&n&l&" + name;
}

std::string DataFormatter::AddLineGraph(const std::string &name, const int &min, const int &max)
{
	sprintf(numstr_, "%i", min);
	sprintf(numstr__, "%i", max);
	return std::string(numstr___) + "&n&l&" + name + "&" + numstr_ + "&" + numstr__;
}

std::string DataFormatter::AddHeatmap(const std::string &name, const unsigned int &sizex, const unsigned int &sizey, const int &min, const int &max)
{
	sprintf(numstr_, "%i", sizex);
	sprintf(numstr__, "%i", sizey);
	sprintf(numstr___, "%i", min);
	sprintf(numstr____, "%i", max);
	return "0&n&h&" + name + "&" + numstr_ + "&" + numstr__ + "&" + numstr___ + "&" + numstr____;
}

std::string DataFormatter::AddHeatmap(const std::string &name, const unsigned int &sizex, const unsigned int &sizey)
{
	sprintf(numstr_, "%i", sizex);
	sprintf(numstr__, "%i", sizey);
	return "0&n&h&" + name + "&" + numstr_ + "&" + numstr__;
}

std::string DataFormatter::AddRepeatedMessage()
{
	sprintf(numstr_____, "%i", rep_string_++);
	return std::string(numstr_____) + "&m";
}

uint64_t DataFormatter::GetAndIncrementPacketIdx()
{
	return pkt_idx_++;
}

void DataFormatter::ResetIdx()
{
	rep_string_ = 0;
}

DataFormatter::~DataFormatter()
{
}
