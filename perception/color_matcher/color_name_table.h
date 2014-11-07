#ifndef COLORNAMETABLE_H
#define COLORNAMETABLE_H
#include <vector>
#include <map>
#include <boost/unordered_map.hpp>

// Static dividevalue
namespace {
const float divideValue = 31 / (251.5 - 3.5);
//(31 / (251.5 - 3.5)) * (251.5-3.5)
}
/**
 * @brief A Point corresponding to the color names map, truncates and performs hashing
 */
struct ColorNamePoint
{
    /**
     * @brief Constructor quantizes and truncates to ints
     * @param r (0..255)
     * @param g (0..255)
     * @param b (0..255)
     */
    explicit ColorNamePoint(float r, float g, float b)
        : r_((r-3.5f) * divideValue), g_((g-3.5f) * divideValue), b_((b-3.5f) * divideValue)
    {}

    /**
     * @brief Constructor quantizes to correct values
     * @param r (0..255)
     * @param g (0..255)
     * @param b (0..255)
     */
    explicit ColorNamePoint(uint8_t r, uint8_t g, uint8_t b)
        : r_((r-3.5f) * divideValue), g_((g-3.5f) * divideValue), b_((b-3.5f) * divideValue)
    {}

    int r_, g_, b_;
};

/// Namespace for ColorNames enum, so that it does not pollute global namespace
namespace ColorNames
{

enum Color
{
    Black = 10, Blue = 1, Brown = 2, Grey = 9,
    Green = 3, Orange = 0, Pink = 4, Purple = 5,
    Red = 6, White = 8, Yellow = 7
};

/**
 * @brief Convert color enum to std::string
 * @param color
 * @return
 */
std::string colorToString(Color color);

Color stringToColor(std::string name);

/// Color enun to RGB
void colorToRGB(Color color, int& r, int& g, int& b);

}

/**
 * @brief Container for containing a probability associated with a name and an index
 */
struct ColorProbability
{
    ColorProbability(ColorNames::Color color, float probality)
        : color_(color),  probality_(probality)
    {}

    ColorNames::Color color_;
    float probality_;

    /// Get the color name string
    std::string name( ) const { return ColorNames::colorToString(color_); }
    /// Get the associated probability
    float probability() const { return probality_; }
    /// Get the color names enum
    ColorNames::Color color() const { return color_;}

};

/// Equals operator needed for boost unordered map
bool operator==(const ColorNamePoint &a, const ColorNamePoint &b);

/// hash function needed for unordered map, uses boost hash_combine
std::size_t hash_value(const ColorNamePoint &pt);


typedef boost::unordered_map<ColorNamePoint, std::vector<ColorProbability> > ColorNamesMap;


// ----------------------------------------------------------------------------


/**
 * @brief Contains methods for getting the probabilistic color values
 */
class ColorNameTable
{
public:
    /// singleton instance method
    static ColorNameTable& instance(bool withConditionals = false);

    /**
     * @brief get associated probabilites
     * @param R
     * @param G
     * @param B
     * @return ColorNameProbabilities structure
     */
    std::vector<ColorProbability> getProbabilities(uint8_t r, uint8_t g, uint8_t b) const { return map_.at(ColorNamePoint(r, g ,b)); }

    /// Get a vector probabilities for a specific color name value
    std::vector<ColorProbability> getProbabilities(const ColorNamePoint& pt) const { return map_.at(pt); }

    /// Get the conditional probilites for all color name values
    std::map<ColorNames::Color, std::map<ColorNames::Color, float> > getConditionals() const { return conditionals_; }

    bool load_config(const std::string& path);

private:
    ColorNamesMap map_;

    /// Conditional probs P(C_x | L_y) where x != n
    std::map<ColorNames::Color, std::map<ColorNames::Color, float> > conditionals_;

    /// Associated colors and indexes where it is the dominant label
    std::map<ColorNames::Color, std::vector<ColorNamePoint> > indexes_;

    /// private singleton constructor
    ColorNameTable(bool withConditionals = false);

    /// Fill the map with precomputed values
    bool create(const std::string& path, bool withConditionals = false);

    /// Calculate P(C_x | L_y) probabilities
    void calculateConditionals();

    /**
     * @brief Helper method for createing a ColorProbability struct, ensures a sorted vector
     * @return a vector of Color probability for the given probabilies
     */
    static std::vector<ColorProbability> createColorProbability(float black,
                                                                float blue,
                                                                float brown,
                                                                float grey,
                                                                float green,
                                                                float orange,
                                                                float pink,
                                                                float purple,
                                                                float red,
                                                                float white,
                                                                float yellow);



};

#endif // COLORNAMETABLE_H
