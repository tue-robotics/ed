#include <fstream>
#include <iostream>
#include <list>

#include <vocabulary_tree/simple_kmeans.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>

#include <siftfast/siftfast.h>

#include <dirent.h>
#include <cmath>
#include <sys/stat.h>
#include <ANN/ANN.h>
#include <math.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <time.h>

//#include "common.h"

// TYPE DEFS
typedef Eigen::Matrix<float, 1, 128> Feature;
typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;

// DOCUMENTINFO CLASS
class DocumentInfo
{
    private:
        bool delete_document;
    public:
        vt::Document* document;
        std::string name;
        DocumentInfo();
        DocumentInfo(vt::Document* document, std::string& name);
        ~DocumentInfo();
        void write (std::ostream& out);
        void read(std::istream& in);
};

// ----------------------------------------------------------------------------------------------------

DocumentInfo::DocumentInfo() :
    delete_document(false) {
}

// ----------------------------------------------------------------------------------------------------

DocumentInfo::DocumentInfo(vt::Document* document, std::string& name) :
    delete_document(false), document(document), name(name) {
}

// ----------------------------------------------------------------------------------------------------

DocumentInfo::~DocumentInfo() {
    if (delete_document)
        delete[] document;
}

// ----------------------------------------------------------------------------------------------------

void DocumentInfo::write(std::ostream& out) {
    size_t length = name.length();
    out.write((char*) &length, sizeof(size_t));
    out.write(name.c_str(), name.length());
    size_t doc_length = document->size();
    out.write((char*) &doc_length, sizeof(size_t));
    out.write((char*) &(document->at(0)), doc_length * sizeof(vt::Word));
}

// ----------------------------------------------------------------------------------------------------

void DocumentInfo::read(std::istream& in) {
    size_t length;
    in.read((char*) &length, sizeof(size_t));
    char* name = new char[length + 1];
    in.read(name, length);
    name[length] = 0;
    this->name.assign(name);
    size_t doc_length;
    in.read((char*) &doc_length, sizeof(size_t));
    document = new vt::Document(doc_length);
    in.read((char*) &document->at(0), doc_length * sizeof(vt::Word));
    this->delete_document = true;
    delete[] name;
}

// FUNCTION HEADERS
Keypoint extract_keypoints(IplImage *image, bool frames_only = false);
void trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& images, bool onlySaveImages = false);
void process_file(std::string& filename, std::vector<FeatureVector>& images, bool onlySaveImages = false);
void build_database(std::string directory);
void process_images(std::string directory);
void save_database_without_tree(std::string& directory);
void save_database(std::string& directory);

// CONFIGURATION VARIABLES
std::string moduleName_;
int votes_count;
int tree_k;
int tree_levels;
int min_cluster_size;
int object_id;

// DATABASE
std::vector<vt::Document> docs;
vt::TreeBuilder<Feature> tree_builder;
vt::VocabularyTree<Feature> tree;
vt::Database* db;
std::vector<std::string> image_names;
std::map<int, DocumentInfo*> documents_map;

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if( argc != 3)
    {
        std::cout << "Usage:\n\n\t odu_finder_db_builder <images_directory> <database_directory>\n\n";
        return 1;
    }

    std::string images_directory = argv[1];
    std::string database_directory = argv[2];

    tree_builder = vt::TreeBuilder<Feature> (Feature::Zero());

    moduleName_ = "odu_finder_db_builder";
    tree_k = 5;
    tree_levels = 5;

    std::cout << "[" << moduleName_ << "] " << "Building database..." << std::endl;
    build_database(images_directory);
    std::cout << "[" << moduleName_ << "] " << "Saving database..." << std::endl;
    save_database(database_directory);

    return 0;
}

// ----------------------------------------------------------------------------------------------------

void build_database(std::string directory) {
    std::vector<FeatureVector> images;

    std::cout << "[" << moduleName_ << "] " << "Scanning directories" << std::endl;
    trace_directory(directory.c_str(), "", images);

    std::cout << "[" << moduleName_ << "] " << "Preparing features for the tree..." << std::endl;

    FeatureVector all_features;
    for (unsigned int i = 0; i < images.size(); ++i)
        for (unsigned int j = 0; j < images[i].size(); ++j)
            all_features.push_back(images[i][j]);

    std::cout << "[" << moduleName_ << "] " << "Building a tree with " << all_features.size() << " nodes..." << std::endl;

    tree_builder.build(all_features, tree_k, tree_levels);
    tree = tree_builder.tree();

    std::cout << "[" << moduleName_ << "] " << "Creating the documents..." << std::endl;

    docs.resize(images.size());

    for (unsigned int i = 0; i < images.size(); ++i) {
        //printf("\tImage %d\n", i);
        for (unsigned int j = 0; j < images[i].size(); ++j) {
            //printf("\t\tFeature %d\n", j);
            docs[i].push_back(tree.quantize(images[i][j]));
        }
    }

    std::cout << "[" << moduleName_ << "] " << "Creating database..." <<std::endl;

    db = new vt::Database(tree.words());

    std::cout << "[" << moduleName_ << "] " << "Populating the database with the documents..." << std::endl;

    for (unsigned int i = 0; i < images.size(); ++i) {
        documents_map[db->insert(docs[i])] = new DocumentInfo(&(docs[i]), image_names[i]);
    }

    std::cout << "[" << moduleName_ << "] " << "Training database..." << std::endl;
    db->computeTfIdfWeights(1);

    std::cout << "[" << moduleName_ << "] " << "Database created!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void process_images(std::string directory) {
    std::vector<FeatureVector> images;
    trace_directory(directory.c_str(), "", images, true);
}

// ----------------------------------------------------------------------------------------------------

void save_database_without_tree(std::string& directory) {
    std::cout << "[" << moduleName_ << "] " << "Saving documents..." << std::endl;

    std::string documents_file(directory);
    documents_file.append("/images.documents");
    std::ofstream out(documents_file.c_str(), std::ios::out | std::ios::binary);
    size_t map_size = documents_map.size();
    out.write((char*) &map_size, sizeof(size_t));
    std::map<int, DocumentInfo*>::iterator iter;

    for (iter = documents_map.begin(); iter != documents_map.end(); ++iter) {
        // TODO: sometimes the last iter->second->document has size 0, this is just a quick fix
        if (iter->second->document->size() > 0){
            out.write((char*) &iter->first, sizeof(int));
            iter->second->write(out);
        }
    }

    std::cout << "[" << moduleName_ << "] " << "Saving weights..." << std::endl;

    std::string weights_file(directory);
    weights_file.append("/images.weights");
    db->saveWeights(weights_file.c_str());
    out.close();
}

// ----------------------------------------------------------------------------------------------------

void save_database(std::string& directory) {
    std::cout << "[" << moduleName_ << "] " << "Saving the tree..." << std::endl;
    std::string tree_file(directory);
    tree_file.append("/images.tree");
    tree.save(tree_file.c_str());
    save_database_without_tree(directory);
}

// ----------------------------------------------------------------------------------------------------

void trace_directory(const char* dir, const char* prefix, std::vector<FeatureVector>& images, bool onlySaveImages) {

    std::cout << "[" << moduleName_ << "] " << "Tracing directory: " << dir << std::endl;

    DIR *pdir = opendir(dir);
    struct dirent *pent = NULL;

    if (pdir == NULL) {
        std::cout << "[" << moduleName_ << "] " << "ERROR! Directory " << dir << " not found" << std::endl;
        return;
    }

    while ((pent = readdir(pdir))) {
        if (strcmp(pent->d_name, ".") != 0 && strcmp(pent->d_name, "..") != 0
                && strcmp(pent->d_name, "IGNORE") != 0 && strcmp(pent->d_name, ".svn") != 0) {

            std::string short_filename(prefix);
            short_filename.append(pent->d_name);

            std::string filename(dir);
            filename.append(pent->d_name);

            struct stat st_buf;
            if (lstat(filename.c_str(), &st_buf) == -1) {
                std::cout << "[" << moduleName_ << "] " << "ERROR: Invalid file name " << filename.c_str() << std::endl;
                std::cout << "[" << moduleName_ << "] " << "Exiting" << std::endl;
                exit(2);
            }

            // if the item is a directory, go recursive
            if (S_ISDIR(st_buf.st_mode)) {
                filename.append("/");
                short_filename.append("/");
                trace_directory(filename.c_str(), short_filename.c_str(), images, onlySaveImages);
            } else {
                // else process it
                std::string extension (short_filename.substr(short_filename.find_last_of(".") + 1));

                if(extension.compare("png") == 0 || extension.compare("jpg") == 0 ||
                   extension.compare("PNG") == 0 || extension.compare("JPG") == 0) {
                    process_file(filename, images, onlySaveImages);
                    image_names.push_back(short_filename);
                } else {
                    std::cout << "[" << moduleName_ << "] " << "Not an image, skipping file " << filename << std::endl;
                }
            }
        }
    }
    closedir(pdir);
}

// ----------------------------------------------------------------------------------------------------

void process_file(std::string& filename, std::vector<FeatureVector>& images, bool onlySaveImages) {
    std::cout << "[" << moduleName_ << "] " << "Processing file " << filename.c_str() << "..." << std::endl;

    IplImage *image = cvLoadImage((char*) filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    Keypoint keypoints = extract_keypoints(image);

    std::cout << "[" << moduleName_ << "] " << "Keypoints extracted" << std::endl;

    FeatureVector features;
    Keypoint p = keypoints;
    int count = 0;

    while (p != NULL) {
        Feature f(p->descrip);
        features.push_back(f);
        p = p->next;
        ++count;
    }

    if (!onlySaveImages)
        images.push_back(features);
    else {
        IplImage *colour_image = cvLoadImage((char*) filename.c_str());
        p = keypoints;
        while (p != NULL) {
            cvCircle(colour_image, cvPoint((int) (p->col), (int) (p->row)), 3, cvScalar(255, 255, 0));
            p = p->next;
        }
        cvSaveImage((char*) filename.c_str(), colour_image);
        cvReleaseImage(&colour_image);
    }

    cvReleaseImage(&image);
    FreeKeypoints(keypoints);
    std::cout << "[" << moduleName_ << "] " << "Done! " << count << " features found!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

Keypoint extract_keypoints(IplImage *image, bool frames_only) {

    Image sift_image = CreateImage(image->height, image->width);

    for (int i = 0; i < image->height; ++i) {
        uint8_t* pSrc = (uint8_t*) image->imageData + image->widthStep * i;
        float* pDst = sift_image->pixels + i * sift_image->stride;
        for (int j = 0; j < image->width; ++j)
            pDst[j] = (float) pSrc[j] * (1.0f / 255.0f);
    }

    Keypoint keypoints;
    if (frames_only)
        keypoints = GetKeypointFrames(sift_image);
    else
        keypoints = GetKeypoints(sift_image);
    DestroyAllImages();
    return keypoints;
}
