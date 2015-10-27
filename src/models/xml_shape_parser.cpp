#include "xml_shape_parser.h"

#include <vector>
#include <string>

#include <geolib/datatypes.h>
#include <geolib/CompositeShape.h>

#include <tinyxml.h>

// ----------------------------------------------------------------------------------------------------

std::vector<double> parseArray(const TiXmlElement* xml_elem)
{
    std::string txt = xml_elem->GetText();

    std::vector<double> v;

    std::string word;
    std::stringstream stream(txt);
    while(getline(stream, word, ' '))
    {
        float f = 0;
        std::istringstream istr(word);
        istr >> f;
        v.push_back(f);
    }

    return v;
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr parseXMLShape(const std::string& filename, std::string& error)
{
    std::stringstream s_error;

    TiXmlDocument doc(filename);
    doc.LoadFile();

    if (doc.Error())
    {
        s_error << "While parsing '" << filename << "': " << std::endl << std::endl
                << doc.ErrorDesc() << " at line " << doc.ErrorRow() << ", col " << doc.ErrorCol() << std::endl;
        error = s_error.str();
        return geo::ShapePtr();
    }

    const TiXmlElement* model_xml = doc.FirstChildElement("model");
    if (!model_xml)
    {
        s_error << "Could not find 'model' element" << std::endl;
        return geo::ShapePtr();
    }

    boost::shared_ptr<geo::CompositeShape> shape(new geo::CompositeShape);

    const TiXmlElement* shape_xml = model_xml->FirstChildElement();
    while (shape_xml)
    {
        geo::Pose3D pose = geo::Pose3D::identity();

        // parse properties valid for all shapes
        const TiXmlElement* xyz_xml = shape_xml->FirstChildElement("xyz");
        if (xyz_xml)
        {
            std::vector<double> xyz = parseArray(xyz_xml);
            pose.setOrigin(geo::Vector3(xyz[0], xyz[1], xyz[2]));
        }

        const TiXmlElement* rpy_xml = shape_xml->FirstChildElement("rpy");
        if (rpy_xml)
        {
            std::vector<double> rpy = parseArray(rpy_xml);
            if (fabs(rpy[0]) > 0.0001 || fabs(rpy[1]) > 0.0001 || fabs(rpy[2]) > 0.0001)
            {
                geo::Matrix3 rot;
                rot.setRPY(rpy[0], rpy[1], rpy[2]);
                pose.setBasis(rot);
            }
        }

        std::vector<double> size;
        const TiXmlElement* size_xml = shape_xml->FirstChildElement("size");
        if (size_xml)
            size = parseArray(size_xml);

        std::string shape_type = shape_xml->Value();
        if (shape_type == "box") {
            const TiXmlElement* min_xml = shape_xml->FirstChildElement("min");
            const TiXmlElement* max_xml = shape_xml->FirstChildElement("max");

            if (min_xml && max_xml)
            {
                std::vector<double> min = parseArray(min_xml);
                std::vector<double> max = parseArray(max_xml);

                if (min.size() == 3 && max.size() == 3)
                {
                    shape->addShape(geo::Box(geo::Vector3(min[0], min[1], min[2]),
                                             geo::Vector3(max[0], max[1], max[2])), pose);
                }
            }
            else if (!size.empty())
            {
                geo::Vector3 v_size(size[0], size[1], size[2]);
                shape->addShape(geo::Box(-v_size / 2, v_size / 2), pose);
            }
            else
            {
                s_error << "In definition '" << filename << "': shape '" << shape_type << "' has no size property" << std::endl;
            }

        } else {
            s_error << "In definition '" << filename << "': Unknown shape type: '" << shape_type << "'" << std::endl;
        }

        shape_xml = shape_xml->NextSiblingElement();
    }

    error = s_error.str();
    if (error.empty())
        return shape;

    return geo::ShapePtr();
}
