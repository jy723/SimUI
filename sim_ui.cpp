#include "sim_ui.h"

#include <iostream>
//eigen
#include <Eigen>

#define M_PI 3.1415926535897932346f


SimUI::SimUI(std::string map_file)
	:hdmap_file_(map_file)
{
	loadHDMapXML();
}

SimUI::~SimUI()
{
    for (auto& it : hdmap_) {        
        delete[] it.second.x;
        delete[] it.second.y;
    }
}

const std::map<std::string, MapObject> SimUI::GetHDMap() {
	return hdmap_;
}

const std::vector<Position> SimUI::GetHDMapRegion() {
    std::vector<Position> region;
    region.push_back(map_left_up_);
    region.push_back(map_right_down_);
    return region;
}

void SimUI::loadHDMapXML() {
	
    //std::string map_file = "data/1_hdmap.xml";
	hdmap_.clear();

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(hdmap_file_.c_str()) != XML_SUCCESS){
		doc.PrintError();
        std::cout << "&s:file not exist!"<< hdmap_file_ << std::endl;
        return;
	}

	const tinyxml2::XMLElement* xml_node = doc.RootElement();

    //map_origin
    auto map_origin_node = xml_node->FirstChildElement("map_original_point");
    map_origin_node->QueryDoubleAttribute("lon", &map_origin_point_.lon);
    map_origin_node->QueryDoubleAttribute("lat", &map_origin_point_.lat);
    map_origin_node->QueryDoubleAttribute("height", &map_origin_point_.height);

    //header region
    auto header_node = xml_node->FirstChildElement("header");
    header_node->QueryDoubleAttribute("north", &map_left_up_.lat);
    header_node->QueryDoubleAttribute("west", &map_left_up_.lon);
    header_node->QueryDoubleAttribute("south", &map_right_down_.lat);
    header_node->QueryDoubleAttribute("east", &map_right_down_.lon);
    tfWgs84ToUtm(map_left_up_);
    tfWgs84ToUtm(map_right_down_);

	//openspace
	auto sub_node_openspace = xml_node->FirstChildElement("openspace");
	while (sub_node_openspace) {
		getOpenspace(*sub_node_openspace);
        sub_node_openspace = sub_node_openspace->NextSiblingElement("openspace");
	}

    //road
    auto sub_node_road = xml_node->FirstChildElement("road");
    while (sub_node_road) {
        getRoads(*sub_node_road);
        sub_node_road = sub_node_road->NextSiblingElement("road");
    }

}

void SimUI::getOpenspace(const tinyxml2::XMLElement& xml_node)
{	
	//one openspace
    MapObject map_obj;
    map_obj.id = xml_node.Attribute("id");
    std::string element_name = "cornerGlobal";
    auto sub_node = xml_node.FirstChildElement("outline");// ->FirstChildElement("cornerGlobal");

    getMapObject(map_obj, sub_node, element_name);

    hdmap_.insert(std::make_pair<>(map_obj.id, map_obj));
}

void SimUI::getRoads(const tinyxml2::XMLElement& xml_node) {

    //laneSection
    auto node_lanes_laneSection = xml_node.FirstChildElement("lanes")->FirstChildElement("laneSection");
    while (node_lanes_laneSection) {

        //right
        auto node_laneSection_right = node_lanes_laneSection->FirstChildElement("right");
        if (!node_laneSection_right) {
            std::cout << "node_laneSection_right is null:" << node_lanes_laneSection->FirstChildElement("right") << std::endl;
            node_lanes_laneSection = node_lanes_laneSection->NextSiblingElement("laneSection");
            continue;
        }

        auto node_lanes_right = node_lanes_laneSection->FirstChildElement("right")->FirstChildElement("lane");
        while (node_lanes_right) {
            MapObject map_obj;
            map_obj.id = node_lanes_right->Attribute("uid");
            std::string element_name = "point";
            std::cout << "lane uid:"<< map_obj.id <<std::endl;
            auto node_pointset = node_lanes_right->FirstChildElement("centerLine")->FirstChildElement("geometry")->FirstChildElement("pointSet");

            getMapObject(map_obj, node_pointset, element_name);

            hdmap_.insert(std::make_pair<>(map_obj.id, map_obj));

            node_lanes_right = node_lanes_right->NextSiblingElement("lane");
        }
        //left

        node_lanes_laneSection = node_lanes_laneSection->NextSiblingElement("laneSection");
    }
}

void SimUI::getMapObject(MapObject& map_obj, const tinyxml2::XMLElement* xml_node, const std::string& element_name) {
    //xml_node is a list node that includes several element named element_name
    auto sub_node = xml_node->FirstChildElement(element_name.c_str());
    while (sub_node) {
        Position pos;
        sub_node->QueryDoubleAttribute("x", &pos.lon);
        sub_node->QueryDoubleAttribute("y", &pos.lat);
        sub_node->QueryDoubleAttribute("z", &pos.height);
  
        tfWgs84ToUtm(pos);
        map_obj.points.push_back(pos);

        sub_node = sub_node->NextSiblingElement(element_name.c_str());
    }

    int count = map_obj.points.size();

    map_obj.x = new double[count];
    map_obj.y = new double[count];
    for (int i = 0; i < count; i++) {
        map_obj.x[i] = map_obj.points.at(i).x;
        map_obj.y[i] = map_obj.points.at(i).y;
    }
}

void SimUI::tfWgs84ToUtm(Position& out_point) {

    //map_original_point
    Position map_original_point;
    map_original_point.lon = map_origin_point_.lon * M_PI / 180;
    map_original_point.lat = map_origin_point_.lat * M_PI / 180;
    map_original_point.height = map_origin_point_.height;
    //input_point
    Position input_point;
    input_point.lon = out_point.lon * M_PI / 180;
    input_point.lat = out_point.lat * M_PI / 180;
    input_point.height = out_point.height;

    // assign values
    double f = 1 / 298.257;
    double e = sqrt(2 * f - f * f);
    double a = 6378137.;
    double N0 = a / sqrt(1 - e * e * sin(map_original_point.lat) * sin(map_original_point.lat));
    double N1 = a / sqrt(1 - e * e * sin(input_point.lat) * sin(input_point.lat));

    //
    Eigen::MatrixXd R(3, 3);
    Eigen::MatrixXd M(3, 1);
    Eigen::MatrixXd X(3, 1);

    R(0, 0) = -sin(map_original_point.lon);
    R(0, 1) = cos(map_original_point.lon);
    R(0, 2) = 0;
    R(1, 0) = -sin(map_original_point.lat) * cos(map_original_point.lon);
    R(1, 1) = -sin(map_original_point.lat) * sin(map_original_point.lon);
    R(1, 2) = cos(map_original_point.lat);
    R(2, 0) = cos(map_original_point.lat) * cos(map_original_point.lon);
    R(2, 1) = cos(map_original_point.lat) * sin(map_original_point.lon);
    R(2, 2) = sin(map_original_point.lat);

    M(0, 0) = (N1 + input_point.height) * cos(input_point.lat) * cos(input_point.lon)
        - (N0 + map_original_point.height) * cos(map_original_point.lat) * cos(map_original_point.lon);
    M(1, 0) = (N1 + input_point.height) * cos(input_point.lat) * sin(input_point.lon)
        - (N0 + map_original_point.height) * cos(map_original_point.lat) * sin(map_original_point.lon);
    M(2, 0) = (N1 * (1 - e * e) + input_point.height) * sin(input_point.lat)
        - (N0 * (1 - e * e) + map_original_point.height) * sin(map_original_point.lat);

    X = R * M;

    out_point.x = X(0, 0);
    out_point.y = X(1, 0);
    out_point.z = X(2, 0);
}