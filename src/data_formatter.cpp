#include "data_formatter.h"

DataFormatter::DataFormatter() : graph_index_(0), heatmap_index_(0), pkt_idx_(0), rep_string_(0)
{
}

std::string DataFormatter::AddBarGraph(std::string name)
{
	sprintf(numstr_, "%i", graph_index_++);
	return std::string(numstr_) + "&n&b&" + name;
}

std::string DataFormatter::AddStringLog(color c)
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

std::string DataFormatter::AddLineGraph(std::string name)
{
	sprintf(numstr_, "%i", graph_index_++);
	return std::string(numstr_) + "&n&l&" + name;
}

std::string DataFormatter::AddLineGraph(std::string name, int min, int max)
{
	sprintf(numstr_, "%i", min);
	sprintf(numstr__, "%i", max);
	sprintf(numstr___, "%i", graph_index_++);
	return  std::string(numstr___) + "&n&l&" + name + "&" + numstr_ + "&" + numstr__;
}

std::string DataFormatter::AddHeatmap(std::string name, unsigned int sizex, unsigned int sizey, int min, int max)
{
	sprintf(numstr_, "%i", sizex);
	sprintf(numstr__, "%i", sizey);
	sprintf(numstr___, "%i", min);
	sprintf(numstr____, "%i", max);
	sprintf(numstr_____, "%i", heatmap_index_++);
	return std::string(numstr_____)+ "&n&h&" + name + "&" + numstr_ + "&" + numstr__ + "&" + numstr___ + "&" + numstr____;
}

std::string DataFormatter::AddHeatmap(std::string name, unsigned int sizex, unsigned int sizey)
{
	sprintf(numstr_, "%i", sizex);
	sprintf(numstr__, "%i", sizey);
	sprintf(numstr_____, "%i", heatmap_index_++);
	return std::string(numstr_____)+ "&n&h&" + name + "&" + numstr_ + "&" + numstr__;
}

std::string DataFormatter::AddRepeatedMessage()
{
	sprintf(numstr_____, "%i", rep_string_++);
	return std::string(numstr_____)+ "&m";
}

uint64_t DataFormatter::GetAndIncrementPacketIdx()
{
	return pkt_idx_++;
}

void DataFormatter::ResetIdx()
{
	graph_index_ = 0;
	heatmap_index_ = 0;
	rep_string_ = 0;
}

DataFormatter::~DataFormatter()
{
}
