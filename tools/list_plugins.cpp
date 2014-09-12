#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <stdlib.h> // for getting environment variable

#include <tue/filesystem/crawler.h>

#include <class_loader/class_loader.h>

#include <ed/plugin.h>

// ----------------------------------------------------------------------------------------------------

bool getEnvironmentVariable(const std::string& var, std::string& value)
{
     const char * val = ::getenv(var.c_str());
     if ( val == 0 )
         return false;

     value = val;
     return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Get plugin paths
    std::vector<std::string> plugin_paths;

    std::string ed_plugin_path;
    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_plugin_path))
    {
        std::stringstream ss(ed_plugin_path);
        std::string item;
        while (std::getline(ss, item, ':'))
            plugin_paths.push_back(item);
    }
    else
    {
        std::cout << "Error: Environment variable ED_PLUGIN_PATH not set." << std::endl;
        return 1;
    }

    for(std::vector<std::string>::const_iterator it = plugin_paths.begin(); it != plugin_paths.end(); ++it)
    {
        tue::filesystem::Crawler crawler(*it);
        crawler.setRecursive(true);
        crawler.setListDirectories(false);
        crawler.setListFiles(true);
        crawler.setIgnoreHiddenDirectories(true);
        crawler.setIgnoreHiddenFiles(true);

        tue::filesystem::Path path;
        while (crawler.nextPath(path))
        {
            if (path.extension() == ".so")
            {

                std::cout << path << std::endl;

                // Create plugin
                //class_loader.loadLibrary();
                try
                {
                    class_loader::ClassLoader class_loader(path.string());
                    std::vector<std::string> classes = class_loader.getAvailableClasses<ed::Plugin>();

                    if (!classes.empty())
                    {
                        std::cout << path.filename() << std::endl;
                    }

                } catch (class_loader::LibraryLoadException& e)
                {

                }

                //class_loader.unloadLibrary();
            }
        }
    }

    return 0;
}
