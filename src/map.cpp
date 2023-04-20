#include <map.h>

void Map::push(Tile t)
{
  if (!find(t))
    map[len++] = t;
}

Tile Map::get(Tile t)
{
  for (uint16_t i = 0; i < len; i++)
  {
    if (t.x == map[i].x && t.y == map[i].y)
    {
      return map[i];
    }
  }
  return {0,0,0,0};
}

bool Map::find(Tile t){
  for (uint16_t i = 0; i < len; i++)
  {
    if (t.x == map[i].x && t.y == map[i].y)
    {
      return true;
    }
  }
  return false;
}

void Map::clear()
{
  len = 0;
}

void Map::print()
{
  for (uint16_t i = 0; i < len; i++)
  {
    Serial.print("[");
    Serial.print(i);
    Serial.print("]");
    Serial.print("\t");
    Serial.print(map[i].x);
    Serial.print(",\t");
    Serial.print(map[i].y);
    Serial.print(",\t");
    Serial.print(map[i].a);
    Serial.print(",\t");
    Serial.println(map[i].b);
  } 
}