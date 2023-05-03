#ifndef DATAFORMATTER_H
#define DATAFORMATTER_H

#include <sstream>
#include <string>
#include <iomanip>

#define PRINT(x)   Serial.print("{\"0:s\":\""); Serial.print(x);   Serial.print("\"}");
#define PRINTLN(x)   Serial.print("{\"0:s\":\""); Serial.print(x);   Serial.print("\\n\"}");

enum color
{
	RED = 'r',
	GREEN = 'g',
	BLUE = 'b'
};

class DataFormatter
{
private:
	uint16_t graph_index_;
	uint16_t heatmap_index_;
	std::stringstream tmp_ss_;
	uint64_t pkt_idx_;
	void CleanStreamAndIncrementIndex(uint16_t &idx);

public:
	DataFormatter();
	std::string AddBarGraph(std::string name);
	std::string AddStringLog(color c);
	std::string AddStringLog();
	std::string AddPacketIndex();
	std::string AddLineGraph(std::string name);
	std::string AddLineGraph(std::string name, int64_t min, int64_t max);
	std::string AddHeatmap(std::string name, uint16_t sizex, uint16_t sizey, int32_t min, int32_t max);
	uint64_t GetPacketIdx();
	void ResetIdx();
	~DataFormatter();
};

#endif /* DATAFORMATTER_H */