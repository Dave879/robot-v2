#ifndef DATAFORMATTER_H
#define DATAFORMATTER_H

#include <string>
#include <stdio.h>

#define PRINT(x)                \
	Serial.print("{\"0&s\":\""); \
	Serial.print(x);             \
	Serial.print("\"}");
#define PRINTLN(x)              \
	Serial.print("{\"0&s\":\""); \
	Serial.print(x);             \
	Serial.print("\\n\"}");

enum color
{
	RED = 'r',
	GREEN = 'g',
	BLUE = 'b'
};

class DataFormatter
{
private:
	unsigned int graph_index_;
	unsigned int heatmap_index_;
	unsigned long long pkt_idx_;
	unsigned int rep_string_;
	char numstr_[21];
	char numstr__[21];
	char numstr___[21];
	char numstr____[21];
	char numstr_____[21];

public:
	DataFormatter();
	std::string AddBarGraph(std::string name);
	std::string AddStringLog(color c);
	std::string AddStringLog();
	std::string AddPacketIndex();
	std::string AddRepeatedMessage();
	std::string AddLineGraph(std::string name);
	std::string AddLineGraph(std::string name, int min, int max);
	std::string AddHeatmap(std::string name, unsigned int sizex, unsigned int sizey, int min, int max);
	std::string AddHeatmap(std::string name, unsigned int sizex, unsigned int sizey);
	uint64_t GetAndIncrementPacketIdx();
	void ResetIdx();
	~DataFormatter();
};

#endif /* DATAFORMATTER_H */