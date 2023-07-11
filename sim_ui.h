#ifndef SIMUI_H
#define SIMUI_H

#pragma once

#include <string>
#include <map>
#include <vector>
#include "tinyxml2.h"
using namespace tinyxml2;

typedef struct POSITION_{
    double lat;
    double lon;
    double height;

    double x;
    double y;
    double z;

    double heading;//deg[0-360]
}Position;

typedef struct MAPOBJECT_ {
    std::string id;
    std::string name;
    std::vector<Position> points;
    double* x;
    double* y;
}MapObject;


class SimUI
{
public:
    SimUI(std::string map_file);
    ~SimUI();

    const std::map<std::string, MapObject> GetHDMap();
    /// <summary>
    /// hdmap region limit
    /// </summary>
    /// <returns>first map_left_up point,second map_right_down point</returns>
    const std::vector<Position> GetHDMapRegion();
private:
    void loadHDMapXML();

    /// <summary>
    /// openspace
    /// </summary>
    /// <param name="xml_node"></param>
    void getOpenspace(const tinyxml2::XMLElement& xml_node);

    /// <summary>
    /// 
    /// </summary>
    /// <param name="xml_node"></param>
    void getRoads(const tinyxml2::XMLElement& xml_node);
    
    /// <summary>
    /// convert road and openspace to mapobject
    /// </summary>
    /// <param name="map_obj"></param>
    /// <param name="xml_node"></param>
    /// <param name="element_name"></param>
    void getMapObject(MapObject& map_obj, const tinyxml2::XMLElement* xml_node, const std::string& element_name);


    /// <summary>
    /// convert out_point (lon,lat) to (x,y)
    /// </summary>
    /// <param name="out_point"></param>
    void tfWgs84ToUtm(Position& out_point);

private:
    std::string hdmap_file_;
    std::map<std::string, MapObject> hdmap_;
    Position map_origin_point_;

    Position map_left_up_;
    Position map_right_down_;
};

#endif