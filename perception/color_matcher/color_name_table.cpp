#include "color_name_table.h"
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

///////////////////////////////////////////////// Hashing functions ///////////////////////////////
bool operator==(const ColorNamePoint &a, const ColorNamePoint &b)
{
    return a.r_ == b.r_ && a.g_ == b.g_ && a.b_ == b.b_;
}


std::size_t hash_value(const ColorNamePoint &pt)
{
    std::size_t seed = 0;
    boost::hash_combine(seed, pt.r_);
    boost::hash_combine(seed, pt.g_);
    boost::hash_combine(seed, pt.b_);
    return seed;
}


//////////////////////////////////////////////// Color Enum ///////////////////////////////////////
namespace ColorNames
{

std::string colorToString(Color color)
{
    switch(color)
    {
    case Black:
        return "Black";
        break;
    case Blue:
        return "Blue";
        break;
    case Brown:
        return "Brown";
        break;
    case Grey:
        return "Grey";
        break;
    case Green:
        return "Green";
        break;
    case Orange:
        return "Orange";
        break;
    case Pink:
        return "Pink";
        break;
    case Purple:
        return "Purple";
        break;
    case Red:
        return "Red";
        break;
    case White:
        return "White";
        break;
    case Yellow:
        return "Yellow";
        break;
    default:
        return "None";
    }
}

Color stringToColor(std::string name)
{
    if (name == "Black") return Black;
    if (name == "Blue") return Blue;
    if (name == "Brown") return Brown;
    if (name == "Grey") return Grey;
    if (name == "Green") return Green;
    if (name == "Orange") return Orange;
    if (name == "Purple") return Purple;
    if (name == "Red") return Red;
    if (name == "White") return White;
    if (name == "Yellow") return Yellow;
}

void colorToRGB(Color color, int& r, int& g, int& b)
{
    switch(color)
    {
    case Black:
        r = 50; g = 50; b = 50;
        break;
    case Blue:
        r = 0; g = 0; b = 255;
        break;
    case Brown:
        r = 160; g = 82; b = 45;
        break;
    case Grey:
        r = 128; g = 138; b = 135;
        break;
    case Green:
       r = 0; g = 255; b = 0;
        break;
    case Orange:
        r=255; g= 140; b=0;
        break;
    case Pink:
       r=255; g=182; b=193;
        break;
    case Purple:
        r=238; g= 0; b=238;
        break;
    case Red:
        r=255; g=0; b=0;
        break;
    case White:
        r=255; g=255; b=255;
        break;
    case Yellow:
        r=255; g=255; b=0;
        break;
    default:
        r=0; g=0; b=0;
    }

}

}


///////////////////////////////////////////////// Color tables ////////////////////////////////////
ColorNameTable::ColorNameTable(bool withConditionals)
{
//    create("bla", withConditionals);
}

///////////////////////////////////////////////// Map Value ///////////////////////////////////////
bool compareColorProbabilities(const ColorProbability& a, const ColorProbability& b)
{
    return a.probality_ > b.probality_;
}


std::vector<ColorProbability> ColorNameTable::createColorProbability(float black, float blue, float brown,
                                                                     float grey, float green, float orange,
                                                                     float pink, float purple, float red,
                                                                     float white, float yellow)
{
    std::vector<ColorProbability> probabilities;

    probabilities.push_back(ColorProbability(ColorNames::Black, black));
    probabilities.push_back(ColorProbability(ColorNames::Blue, blue));
    probabilities.push_back(ColorProbability(ColorNames::Brown, brown));
    probabilities.push_back(ColorProbability(ColorNames::Grey, grey));
    probabilities.push_back(ColorProbability(ColorNames::Green, green));
    probabilities.push_back(ColorProbability(ColorNames::Orange, orange));
    probabilities.push_back(ColorProbability(ColorNames::Pink, pink));
    probabilities.push_back(ColorProbability(ColorNames::Purple, purple));
    probabilities.push_back(ColorProbability(ColorNames::Red, red));
    probabilities.push_back(ColorProbability(ColorNames::White, white));
    probabilities.push_back(ColorProbability(ColorNames::Yellow, yellow));

    //Sort so highest probability comes out on top
    std::sort(probabilities.begin(), probabilities.end(), &compareColorProbabilities);

    return probabilities;
}

ColorNameTable& ColorNameTable::instance(bool withConditionals)
{
    static boost::shared_ptr<ColorNameTable> tableInstance = boost::shared_ptr<ColorNameTable>(new ColorNameTable(withConditionals));
    return *tableInstance;
}


namespace
{

std::vector<std::string>& split(const std::string &s, const char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, const char delim) {
    std::vector<std::string> elems;
    return split(s, delim, elems);
}

}

bool ColorNameTable::load_config(const std::string& path){
    return create(path, false);
}

bool ColorNameTable::create(const std::string& path, bool withConditionals)
{
    //Read from file
    std::ifstream stream;

//    std::cout << "Loading color name table:" << path << std::endl;
    stream.open(path.c_str(), std::ios_base::in);

    //TODO Optimize this later?
    if(stream.is_open())
    {
//        std::cout << "Start reading..." << std::endl;
        std::string line;

        while(std::getline(stream, line))
        {
            std::vector<std::string> lines = split(line, ' ');
            ColorNamePoint point(boost::lexical_cast<float>(lines.at(0)), boost::lexical_cast<float>(lines.at(1)), boost::lexical_cast<float>(lines.at(2)));
            //order of color names: black, blue, brown, grey, green, orange, pink, purple, red, white, yellow

            assert(map_.find(point) == map_.end());

            std::vector<ColorProbability> probabilities = createColorProbability(boost::lexical_cast<float>(lines.at(3)),  // black
                                                                                 boost::lexical_cast<float>(lines.at(4)),  // blue
                                                                                 boost::lexical_cast<float>(lines.at(5)),  // brown
                                                                                 boost::lexical_cast<float>(lines.at(6)),  // grey
                                                                                 boost::lexical_cast<float>(lines.at(7)),  // green
                                                                                 boost::lexical_cast<float>(lines.at(8)),  // orange
                                                                                 boost::lexical_cast<float>(lines.at(9)),  // pink
                                                                                 boost::lexical_cast<float>(lines.at(10)), // purple
                                                                                 boost::lexical_cast<float>(lines.at(11)), // red
                                                                                 boost::lexical_cast<float>(lines.at(12)), // white
                                                                                 boost::lexical_cast<float>(lines.at(13))); //yellow
            map_[point] = probabilities;

            if(withConditionals)
            {
                //Create index for calcuating conditionals
                ColorNames::Color color = probabilities.at(0).color();
                if(indexes_.find(color) != indexes_.end())
                {
                    indexes_[color].push_back(point);
                }
                else
                {
                    indexes_[color] = std::vector<ColorNamePoint>();
                    indexes_[color].push_back(point);
                }
            }
        }
        stream.close();

        if(withConditionals)
            calculateConditionals();
    }
    else{
        return false;
        throw std::runtime_error("Cannot load color names table!");
    }

//    std::cout << "Finished reading..." << std::endl;

    return true;
}


void ColorNameTable::calculateConditionals()
{   
    for(std::map<ColorNames::Color, std::vector<ColorNamePoint> >::iterator it = indexes_.begin(); it != indexes_.end(); ++it)
    {
        std::map<ColorNames::Color, float> conditionals;

        //Initialize all colors to zero
        conditionals[ColorNames::Black] = 0.0f;
        conditionals[ColorNames::Blue] = 0.0f;
        conditionals[ColorNames::Brown] = 0.0f;
        conditionals[ColorNames::Grey] = 0.0f;
        conditionals[ColorNames::Green] = 0.0f;
        conditionals[ColorNames::Orange] = 0.0f;
        conditionals[ColorNames::Pink] = 0.0f;
        conditionals[ColorNames::Purple] = 0.0f;
        conditionals[ColorNames::Red] = 0.0f;
        conditionals[ColorNames::White] = 0.0f;
        conditionals[ColorNames::Yellow] = 0.0f;

        //Iterate over all locations, which are labeled with the given color
        for(std::vector<ColorNamePoint>::iterator idxsIt = it->second.begin(); idxsIt != it->second.end(); ++idxsIt)
        {
            //Acquire probabilities at location
            std::vector<ColorProbability> probabilities = getProbabilities(*idxsIt);

            //Iterate over probabilities and add to accumulator
            for(std::vector<ColorProbability>::iterator probIt = probabilities.begin(); probIt != probabilities.end(); ++probIt)
            {
                const ColorProbability& p = *probIt;
                conditionals[p.color()] += p.probability();
            }

            //Marginalize
            const float factor = conditionals[it->first];

            conditionals[ColorNames::Black] /= factor;
            conditionals[ColorNames::Blue] /= factor;
            conditionals[ColorNames::Brown] /= factor;
            conditionals[ColorNames::Grey] /= factor;
            conditionals[ColorNames::Green] /= factor;
            conditionals[ColorNames::Orange] /= factor;
            conditionals[ColorNames::Pink] /= factor;
            conditionals[ColorNames::Purple] /= factor;
            conditionals[ColorNames::Red] /= factor;
            conditionals[ColorNames::White] /= factor;
            conditionals[ColorNames::Yellow] /= factor;

        }

        conditionals_[it->first] = conditionals;
    }
}


