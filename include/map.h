#pragma once

#include <Arduino.h>

struct Tile{
	int16_t x, y, a, b;
};

class Map{
	Tile map[500];
	uint16_t len = 0;

	public:
	void push(Tile t);
	Tile get(Tile t);
	bool find(Tile t);
	void clear();
	void print();
};