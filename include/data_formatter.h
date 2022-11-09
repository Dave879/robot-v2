#ifndef DATAFORMATTER_H
#define DATAFORMATTER_H

#include <sstream>
#include <string>
#include <iomanip>

class DataFormatter
{
private:
	uint16_t graph_index_;
	std::stringstream tmp_ss_;
public:
	DataFormatter();
	std::string AddBarGraph(std::string name);
	std::string AddLineGraph(std::string name);
	std::string AddHeatmap(std::string name, uint8_t sizex, uint8_t sizey);
	void ResetIdx();
	~DataFormatter();
};

#endif /* DATAFORMATTER_H */